/* USB floppy controller for amiga disks
 *
 * Copyright (C) 2019 Gianluca Renzi (icjtqr@gmail.com)
 * Copyright (C) 2017-2018 Robert Smith (@RobSmithDev)
 * Copyright (C) 2018 John Tsiombikas <nuclear@member.fsf.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this program; if not, see http://www.gnu.org/licenses
 */
/* Ported from arduino to standalone AVR by John Tsiombikas */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

#define INDEX_PORT			PIND
#define INDEX_BIT			0x04

#define WDATA_PORT			PORTD
#define WDATA_BIT			0x08

#define RDATA_PORT			PIND
#define RDATA_BIT			0x10

#define MOTOR_PORT			PORTD
#define MOTOR_ENABLE_BIT	0x20
#define MOTOR_DIR_BIT		0x40
#define MOTOR_STEP_BIT		0x80

#define TRACK0_PORT			PINB
#define TRACK0_BIT			0x01

#define HEADSEL_PORT		PORTB
#define HEAD_SELECT_BIT		0x02

#define WGATE_PORT			PORTC
#define WGATE_BIT			0x01

#define WPROT_PORT			PINC
#define WPROT_BIT			0x02

#define CTS_PORT			PORTC
#define CTS_BIT				0x04

#define LED_PORT			PORTB
#define LED_BIT				0x20


#define BAUDRATE 2000000

#define BAUD_PRESCALLER_NORMAL_MODE      (F_CPU / 16 / BAUDRATE - 1)
#define BAUD_PRESCALLER_DOUBLESPEED_MODE (F_CPU /  8 / BAUDRATE - 1)
/* We're using double speed mode */
#define UART_USE_DOUBLESPEED_MODE

/* Motor directions for PIN settings */
#define MOTOR_TRACK_DECREASE   1
#define MOTOR_TRACK_INCREASE   0
#define MOTOR_DIR(x) ((x) ? (MOTOR_PORT |= MOTOR_DIR_BIT) : (MOTOR_PORT &= ~MOTOR_DIR_BIT))

/* Paula on the Amiga used to find the SYNC WORDS and then read 0x1900 further WORDS.
 * A dos track is 11968 bytes in size, theritical revolution is 12800 bytes.
 * Paula assumed it was 12868 bytes, so we read that, plus thre size of a sectors
 */
#define RAW_TRACKDATA_LENGTH    (0x1900 * 2 + 0x440)
/* For the HD (1.4 MBytes) Disks the amount of data should be about 26688: */
#define RAW_HD_TRACKDATA_LENGTH (0x1900 * 2 * 2 + 0x440)

static void smalldelay(unsigned long delay_time);
static void step_direction_head(void);
static void step_direction_head_fast(void);
static void prep_serial_interface(void);
static inline unsigned char read_byte_from_uart(void);
static inline void write_byte_to_uart(const char value);
static void setup(void);
static void run_diagnostic(void);
static int goto_track0(void);
static int goto_track_x(void);
static void write_track_from_uart(void);
static void write_track_from_uart_hd(void);
static void erase_track(void);
static void erase_track_hd(void);
static void read_track_data_fast(void);
static void read_track_data_fast_hd(void);
static char *i2a(unsigned int i, char *a, unsigned r);
static void measure_track_data(void);
static void loop(void);

static int current_track = -1; /* The current track that the head is over. Starts with -1 to identify an unknown head position. */
static int drive_enabled = 0; /* If the drive has been switched on or not */
static int in_write_mode = 0; /* If we're in WRITING mode or not */

/* Where there should be a HD Disk been read (1) or a DD and SD Disk (0).*/
static int disktypeHD = 0;

int main(void)
{
	setup();
	for(;;) {
		loop();
	}
	return 0;
}

static inline void led_blink(int delay)
{
	PORTB |= LED_BIT;
	smalldelay(delay);
	PORTB &= ~LED_BIT;
	smalldelay(delay);
}

static inline void led_off(void)
{
	PORTB &= ~LED_BIT;
}

static void notify_ready(int delay)
{
	int i;
	char ready[32];

	sprintf(ready, "AMIGA IS READY TO ROLL!\n\r");

	for (i = 0; i < strlen(ready); i++)
	{
		write_byte_to_uart(ready[i]);
		led_blink(delay);
	}
}

/* Because we turned off interrupts delay() doesnt work! */
static void smalldelay(unsigned long delay_time)
{
	unsigned long i;

	delay_time *= (F_CPU / 9000);

	for(i=0; i<delay_time; ++i) {
		asm volatile("nop");
	}
}

/* Step the head once.  This seems to be an acceptable speed for the head */
static void step_direction_head(void)
{
	smalldelay(5);
	MOTOR_PORT &= ~MOTOR_STEP_BIT;
	smalldelay(5);
	MOTOR_PORT |= MOTOR_STEP_BIT;
}

/* Step the head once. Do it a bit faster... */
static void step_direction_head_fast(void)
{
	smalldelay(4);
	MOTOR_PORT &= ~MOTOR_STEP_BIT;
	smalldelay(3);
	MOTOR_PORT |= MOTOR_STEP_BIT;
}

/* Prepare serial port - We dont want to use the arduino serial library as we
 * want to use faster speeds and no serial interrupts
 */
static void prep_serial_interface(void)
{
#ifdef UART_USE_DOUBLESPEED_MODE
	UBRR0H = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE >> 0);
	UCSR0A |= 1<<U2X0;
#else
	UBRR0H = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE >> 0);
	UCSR0A &= ~(1<<U2X0);
#endif

	/*
	 * UCSROA is a status register only (apart from U2Xn):
	 * • Bit 7 – RXCn: USART Receive Complete
	 * • Bit 6 – TXCn: USART Transmit Complete
	 * • Bit 5 – UDREn: USART Data Register Empty
	 * • Bit 4 – FEn: Frame Error
	 * • Bit 3 – DORn: Data OverRun/
	 * • Bit 2 – UPEn: USART Parity Error
	 * • Bit 1 – U2Xn: Double the USART Transmission Speed
	 * • Bit 0 – MPCMn: Multi-processor Communication Mode
	 */

	UCSR0B  = (0<<RXCIE0)  |   /* Disable ReceiveCompleteInteruptEnable */
			  (0<<TXCIE0)  |   /* Disable TransmitCompleteInteruptEnable */
			  (0<<UDRIE0)  |   /* Disable UsartDataRegisterEmptyInteruptEnable */
			  (1<<RXEN0)   |   /* Enable RX */
			  (1<<TXEN0)   |   /* Enable TX */
			  (0<<UCSZ02)  ;   /* Clear the 9-bit character mode bit */

	UCSR0C =  (0<<UMSEL01) | (0<<UMSEL00) |  /* UsartModeSelect - Asynchronous (00=Async, 01=Sync, 10=Reserved, 11=Master SPI) */
			  (0<<UPM01)   | (0<<UPM00)   |  /* UsartParatyMode - Disabled  (00=Off, 01=Reserved, 10=Even, 11=Odd) */
			  (0<<USBS0)   |                 /* UsartStopBitSelect (0=1 Stop bit, 1 = 2Stop Bits) */
			  (1<<UCSZ01)  | (1<<UCSZ00);    /* UsartCharacterSiZe  - 8-bit (00=5Bit, 01=6Bit, 10=7Bit, 11=8Bit, must be 11 for 9-bit) */
}

/* Directly read a byte from the UART0 */
static inline unsigned char read_byte_from_uart(void)
{
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

/* Directly write a byte to the UART0 */
static inline void write_byte_to_uart(const char value)
{
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = value;
}

static void setup(void)
{
	DDRB = 0x22;	/* outputs: 1 (head sel), 5 (act LED) */
	DDRC = 0xf5;	/* outputs: 0 (wr.gate), 2 (cts) */
	DDRD = 0xe8;	/* outputs: 3 (write), 5 (motor en), 6 (dir), 7 (step) */

	/* Do these right away to prevent the disk being written to */
	PORTC = WGATE_BIT | WPROT_BIT;	/* write gate off and pullup on pin 1 (wprot) */
	PORTD = WDATA_BIT | RDATA_BIT;	/* write data hight and pullup on pin 4 (read) */
	PORTB = TRACK0_BIT;				/* pullup on track0 detect */

	MOTOR_PORT |= MOTOR_ENABLE_BIT;
	HEADSEL_PORT &= ~HEAD_SELECT_BIT;

	/* Disable all interrupts - we dont want them! */
	cli();
	TIMSK0=0;
	TIMSK1=0;
	TIMSK2=0;
	PCICR = 0;
	PCIFR = 0;
	PCMSK0 = 0;
	PCMSK2 = 0;
	PCMSK1 = 0;
	/* Setup the USART */
	prep_serial_interface();

	/* Notify with the ACT LED we are ready to roll!! */
	notify_ready(50);

	led_off();
}

static void run_diagnostic(void)
{
	int j, i, state1, state2, res;
	char test;

	test = read_byte_from_uart();
	switch(test) {
	case '1':	/* turn off CTS */
		CTS_PORT &= ~CTS_BIT;
		write_byte_to_uart('1');
		read_byte_from_uart();
		write_byte_to_uart('1');
		break;

	case '2':	/* turn on CTS */
		CTS_PORT |= CTS_BIT;
		write_byte_to_uart('1');
		read_byte_from_uart();
		write_byte_to_uart('1');
		break;

	case '3':	/* index pulse test (with timeout) */
		state1 = state2 = res = 0;

		/* At the 300 RPM (5 turns per second) this runs at, this loop needs to
		 * run a few times to check for index pulses. This runs for approx 1 sec
		 */
		for(j=0; j<20; j++) {
			for(i=0; i<60000; i++) {
				if(INDEX_PORT & INDEX_BIT) state1 = 1; else state2 = 1;
				if(state1 && state2) break;
			}
			if (state1 && state2) break;
		}
		write_byte_to_uart((state1 && state2) ? '1' : '0');
		break;

	case '4':	/* data pulse test (with timeout) */
		state1 = state2 = res = 0;

		for(j=0; j<20; j++) {
			for(i=0; i<60000; i++) {
				if(RDATA_PORT & RDATA_BIT) state1 = 1; else state2 = 1;
				if(state1 && state2) break;
			}
			if (state1 && state2) break;
		}
		write_byte_to_uart((state1 && state2) ? '1' : '0');
		break;

	default:
		write_byte_to_uart('0');
		break;
	}
}

/* Rewinds the head back to track 0 */
static int goto_track0(void)
{
	int steps = 0;

	/* Set the direction to go backwards */
	MOTOR_DIR(MOTOR_TRACK_DECREASE);
	while((TRACK0_PORT & TRACK0_BIT)) {
		step_direction_head_fast();	/* Keep moving the head until we see the TRACK 0 detection pin */
		steps++;
		if(steps > 170) {
			/* we've stepped twice as much as the maximum possible steps needed, and still
			 * haven't reached track 0
			 */
			return -1;
		}
	}
	current_track = 0;	/* Reset the track number */
	return 0;
}

/* Goto a specific track.  During testing it was easier for the track number
 * to be supplied as two ASCII characters, so I left it like this
 */
static int goto_track_x(void)
{
	int track;
	unsigned char track1, track2;

	/* Read the bytes */
	track1 = read_byte_from_uart();
	track2 = read_byte_from_uart();

	/* Validate */
	if((track1 < '0') || (track1 > '9')) return 0;
	if((track2 < '0') || (track2 > '9')) return 0;

	/* Calculate target track and validate */
	track = ((track1 - '0') * 10) + (track2 - '0');
	if(track < 0) return 0;
	if(track > 81) return 0; /* yes amiga could read track 81! */

	/* Exit if its already been reached */
	if(track == current_track) return 1;

	/* And step the head until we reach this track number */
	if(current_track < track) {
		MOTOR_DIR(MOTOR_TRACK_INCREASE);	/* move out */
		while(current_track < track) {
			step_direction_head();
			current_track++;
		}
	} else {
		MOTOR_DIR(MOTOR_TRACK_DECREASE);	/* move in */
		while(current_track > track) {
			step_direction_head();
			current_track--;
		}
	}

	return 1;
}

/* 256 byte circular buffer -
 * don't change this, we abuse the unsigned char to overflow back to zero!
 */
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_BUFFER_START (SERIAL_BUFFER_SIZE - 16)
static unsigned char SERIAL_BUFFER[SERIAL_BUFFER_SIZE];


#define CHECK_SERIAL()		if (UCSR0A & (1 << RXC0)) {						\
								SERIAL_BUFFER[serial_write_pos++] = UDR0;	\
								serial_bytes_in_use++;						\
							}												\
							if (serial_bytes_in_use < SERIAL_BUFFER_START)	\
								CTS_PORT &= ~CTS_BIT;						\
							else CTS_PORT |= CTS_BIT;						\

/* Small Macro to write a '1' pulse to the drive if a bit is set based on the supplied bitmask */
#define WRITE_BIT(value, bitmask)	if (current_byte & bitmask)  {			\
										while(TCNT2 < value) {};			\
										WDATA_PORT &= ~WDATA_BIT;			\
									} else {								\
										while(TCNT2 < value) {};			\
										WDATA_PORT |= WDATA_BIT;			\
									}

/* Write a track to disk from the UART -
 * the data should be pre-MFM encoded raw track data where '1's are the
 * pulses/phase reversals to trigger
 */
static void write_track_from_uart(void)
{
	unsigned int i, serial_bytes_in_use;
	unsigned char high_byte, low_byte, wait_for_index;
	unsigned char serial_read_pos, serial_write_pos;
	unsigned short num_bytes;
	register unsigned int a;
	register unsigned char current_byte;

	/* Configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;				/* No physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* Prescale = 1 */

	/* Check if its write protected.
	 * You can only do this after the write gate has been pulled low
	 */
	if((WPROT_PORT & WPROT_BIT) == 0) {
		write_byte_to_uart('N');
		WGATE_PORT |= WGATE_BIT;
		return;
	} else write_byte_to_uart('Y');

	/* Find out how many bytes they want to send */
	high_byte = read_byte_from_uart();
	low_byte = read_byte_from_uart();
	wait_for_index = read_byte_from_uart();
	CTS_PORT |= CTS_BIT;	/* stop any more data coming in! */

	num_bytes = (((unsigned short)high_byte) << 8) | low_byte;

	write_byte_to_uart('!');

	/* Signal we're ready for another byte to come */
	CTS_PORT &= ~CTS_BIT;

	/* Fill our buffer to give us a head start */
	for(i=0; i<SERIAL_BUFFER_START; i++) {
		/* Wait for it */
		while(!(UCSR0A & (1 << RXC0))) {};
		/* Save the byte */
		SERIAL_BUFFER[i] = UDR0;
	}

	/* Stop more bytes coming in, although we expect one more */
	CTS_PORT |= CTS_BIT;

	/* Setup buffer parameters */
	serial_read_pos = 0;
	serial_write_pos = SERIAL_BUFFER_START;
	serial_bytes_in_use = SERIAL_BUFFER_START;
	LED_PORT |= LED_BIT;

	/* Enable writing */
	WGATE_PORT &= ~WGATE_BIT;

	/* While the INDEX pin is high wait.  Might as well write from the start of the track */
	if(wait_for_index)
		while(INDEX_PORT & INDEX_BIT) {};

	/* Reset the counter, ready for writing */
	TCNT2 = 0;

	/* ideally we'd use an ISR here, but there's just too much overhead even with naked
	 * ISRs to do this (preserving registers, etc)
	 */
	for(a=0; a<num_bytes; a++) {
		/* Should never happen, but we'll wait here if theres no data */
		if(serial_bytes_in_use < 1) {
			/* This can't happen and causes a write failure */
			LED_PORT &= ~LED_BIT;
			/* Thus means buffer underflow. PC wasn't sending us data fast enough */
			write_byte_to_uart('X');
			WGATE_PORT |= WGATE_BIT;
			TCCR2B = 0;   /* No Clock (turn off) */
			return;
		}

		/* Read a buye from the buffer */
		current_byte = SERIAL_BUFFER[serial_read_pos++];
		serial_bytes_in_use--;

		/* there's a small possibility we actually get back here before TCNT2 overflows
		 * back to zero, causing this to write early.
		 */
		while(TCNT2 >= 240) {};

		/* Now we write the data. Hopefully by the time we get back to the top
		 * everything is ready again
		 */
		WRITE_BIT(0x10, 0x80);
		CHECK_SERIAL();
		WRITE_BIT(0x30, 0x40);
		CHECK_SERIAL();
		WRITE_BIT(0x50, 0x20);
		CHECK_SERIAL();
		WRITE_BIT(0x70, 0x10);
		CHECK_SERIAL();
		WRITE_BIT(0x90, 0x08);
		CHECK_SERIAL();
		WRITE_BIT(0xb0, 0x04);
		CHECK_SERIAL();
		WRITE_BIT(0xd0, 0x02);
		CHECK_SERIAL();
		WRITE_BIT(0xf0, 0x01);
	}
	WGATE_PORT |= WGATE_BIT;

	/* Done! */
	write_byte_to_uart('1');
	LED_PORT &= ~LED_BIT;

	/* Disable the 500khz signal */
	TCCR2B = 0;   /* No Clock (turn off) */
}

// Write a track to disk from the UART - the data should be pre-MFM encoded raw track data where '1's are the pulses/phase reversals to trigger
// THIS CODE IS UNTESTED
static void write_track_from_uart_hd(void)
{
	unsigned int i, serial_bytes_in_use;
	unsigned char high_byte, low_byte, wait_for_index;
	unsigned char serial_read_pos, serial_write_pos;
	unsigned short num_bytes;
	register unsigned int a;
	register unsigned char current_byte;

	/* Configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;			/* No physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* Prescale = 1 */
    
	/* Check if its write protected.
	 * You can only do this after the write gate has been pulled low
	 */
	if((WPROT_PORT & WPROT_BIT) == 0) {
		write_byte_to_uart('N');
		WGATE_PORT |= WGATE_BIT;
		return;
	} else write_byte_to_uart('Y');

	/* Find out how many bytes they want to send */
	high_byte = read_byte_from_uart();
	low_byte = read_byte_from_uart();
	wait_for_index = read_byte_from_uart();
	CTS_PORT |= CTS_BIT;	/* stop any more data coming in! */
    
	num_bytes = (((unsigned short)high_byte) << 8) | low_byte;

	write_byte_to_uart('!');

	/* Signal we're ready for another byte to come */
	CTS_PORT &= ~CTS_BIT;

	/* Fill our buffer to give us a head start */
	for(i=0; i<SERIAL_BUFFER_START; i++) {
		/* Wait for it */
		while(!(UCSR0A & (1 << RXC0))) {};
		/* Save the byte */
		SERIAL_BUFFER[i] = UDR0;
	}

	/* Stop more bytes coming in, although we expect one more */
	CTS_PORT |= CTS_BIT;

	/* Setup buffer parameters */
	serial_read_pos = 0;
	serial_write_pos = SERIAL_BUFFER_START;
	serial_bytes_in_use = SERIAL_BUFFER_START;
	LED_PORT |= LED_BIT;

	/* Enable writing */
	WGATE_PORT &= ~WGATE_BIT;

	/* While the INDEX pin is high wait.  Might as well write from the start of the track */
	if(wait_for_index)
		while(INDEX_PORT & INDEX_BIT) {};

	/* Reset the counter, ready for writing */
	TCNT2 = 0;

	/* ideally we'd use an ISR here, but there's just too much overhead even with naked
	 * ISRs to do this (preserving registers, etc)
	 */
	for(a=0; a<num_bytes; a++) {
		/* Should never happen, but we'll wait here if theres no data */
		if(serial_bytes_in_use < 1) {
			/* This can't happen and causes a write failure */
			LED_PORT &= ~LED_BIT;
			/* Thus means buffer underflow. PC wasn't sending us data fast enough */
			write_byte_to_uart('X');
			WGATE_PORT |= WGATE_BIT;
			TCCR2B = 0;   /* No Clock (turn off) */
			return;
		}

		/* Read a buye from the buffer */
		current_byte = SERIAL_BUFFER[serial_read_pos++];
		serial_bytes_in_use--;

		/* there's a small possibility, looking at the decompiled ASM (and less likely even
		 * with these few extra instructions) we actually might get back here before TCNT2 overflows
		 * back to zero, causing this to write early.
		 */
		while(TCNT2 >= 248) {};

		/* Now we write the data. Hopefully by the time we get back to the top
		 * everything is ready again
		 */
		WRITE_BIT(0x08, 0x80);
		CHECK_SERIAL();
		WRITE_BIT(0x18, 0x40);
		CHECK_SERIAL();
		WRITE_BIT(0x28, 0x20);
		CHECK_SERIAL();
		WRITE_BIT(0x38, 0x10);
		CHECK_SERIAL();
		WRITE_BIT(0x48, 0x08);
		CHECK_SERIAL();
		WRITE_BIT(0x58, 0x04);
		CHECK_SERIAL();
		WRITE_BIT(0x68, 0x02);
		CHECK_SERIAL();
		WRITE_BIT(0x78, 0x01);
		TCNT2 = 248; // a little cheating, but *should* work
	}

	/* Turn off the write head */
	WGATE_PORT |= WGATE_BIT;

	/* Done! */
	write_byte_to_uart('1');
	LED_PORT &= ~LED_BIT;

	/* Disable the 500khz signal */
	TCCR2B = 0;   /* No Clock (turn off) */
}

static void erase_track(void)
{
	register unsigned int i;
	register unsigned char current_byte;

	/* configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;		/* no physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* prescale = 1 */

	/* check if it's write protected */
	if((WPROT_PORT & WPROT_BIT) == 0) {
		write_byte_to_uart('N');
		WGATE_PORT |= WGATE_BIT;
		return;
	}
	write_byte_to_uart('Y');

	LED_PORT |= LED_BIT;

	/* enable writing */
	WGATE_PORT &= ~WGATE_BIT;

	/* reset the counter, ready for writing */
	TCNT2 = 0;
	current_byte = 0xaa;

	/* write complete blank track - at 300rpm, 500kbps, a track takes approx 1/5
	 * second to write. this is roughly 12500 bytes. our RAW read is 13888 bytes,
	 * so we'll use that just to make sure we get every last bit.
	 */
	for(i=0; i<RAW_TRACKDATA_LENGTH; i++) {
		WRITE_BIT(0x10, 0x80);
		WRITE_BIT(0x30, 0x40);
		WRITE_BIT(0x50, 0x20);
		WRITE_BIT(0x70, 0x10);
		WRITE_BIT(0x90, 0x08);
		WRITE_BIT(0xb0, 0x04);
		WRITE_BIT(0xd0, 0x02);
		WRITE_BIT(0xf0, 0x01);
		while(TCNT2 >= 240) {};
	}

	/* turn the write head off */
	WGATE_PORT |= WGATE_BIT;

	/* done! */
	write_byte_to_uart('1');
	LED_PORT &= ~LED_BIT;

	/* disable the 500khz signal */
	TCCR2B = 0;	/* no clock (turn off) */
}

static void erase_track_hd(void)
{
	register unsigned int i;
	register unsigned char current_byte;

	/* configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;		/* no physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* prescale = 1 */

	/* check if it's write protected */
	if((WPROT_PORT & WPROT_BIT) == 0) {
		write_byte_to_uart('N');
		WGATE_PORT |= WGATE_BIT;
		return;
	}
	write_byte_to_uart('Y');

	LED_PORT |= LED_BIT;

	/* enable writing */
	WGATE_PORT &= ~WGATE_BIT;

	/* reset the counter, ready for writing */
	TCNT2 = 0;
	current_byte = 0xaa;

	/* write complete blank track - at 300rpm, 500kbps, a track takes approx 1/5
	 * second to write. this is roughly 12500 bytes. our RAW read is 13888 bytes,
	 * so we'll use that just to make sure we get every last bit.
	 */
	for(i=0; i<RAW_TRACKDATA_LENGTH; i++) {
		WRITE_BIT(0x08,0x80);
		WRITE_BIT(0x18,0x40);
		WRITE_BIT(0x28,0x20);
		WRITE_BIT(0x38,0x10);
		WRITE_BIT(0x48,0x08);
		WRITE_BIT(0x58,0x04);
		WRITE_BIT(0x68,0x02);
		WRITE_BIT(0x78,0x01);
		WRITE_BIT(0x89,0x80);
		WRITE_BIT(0x98,0x40);
		WRITE_BIT(0xA8,0x20);
		WRITE_BIT(0xB8,0x10);
		WRITE_BIT(0xC8,0x08);
		WRITE_BIT(0xD8,0x04);
		WRITE_BIT(0xE8,0x02);
		WRITE_BIT(0xF8,0x01);
		while (TCNT2 >= 248) {};
	}

	/* turn the write head off */
	WGATE_PORT |= WGATE_BIT;

	/* done! */
	write_byte_to_uart('1');
	LED_PORT &= ~LED_BIT;

	/* disable the 500khz signal */
	TCCR2B = 0;	/* no clock (turn off) */
}


/* Read the track using a timings to calculate which MFM sequence has been triggered */
static void read_track_data_fast(void)
{
	register unsigned char data_output_byte, counter, bits;
	long total_bits, target;

	/* Configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;			/* No physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* Prescale = 1 */

	/* First wait for the serial port to be available */
	while(!(UCSR0A & (1 << UDRE0)));

	/* Signal we're active */
	LED_PORT |= LED_BIT;

	/* While the INDEX pin is high wait if the other end requires us to */
	if(read_byte_from_uart()) while(INDEX_PORT & INDEX_BIT);

	/* Prepare the two counter values as follows: */
	TCNT2=0;	   /* Reset the counter */

	data_output_byte = 0;
	total_bits = 0;
	target = (long)RAW_TRACKDATA_LENGTH * 8L;

	while(total_bits < target) {
		for(bits=0; bits<4; bits++) {
			/* Wait while pin is high */

			while(RDATA_PORT & RDATA_BIT);
			counter = TCNT2, TCNT2 = 0;  /* reset */

			data_output_byte <<= 2;

			if(counter < 80) {
				data_output_byte |= 0x01;
				total_bits += 2;
			} else if(counter > 111) {
				/* this accounts for just a '1' or a '01' as two '1' arent allowed in a row */
				data_output_byte |= 0x03;
				total_bits += 4;
			} else {
				data_output_byte |= 0x02;
				total_bits += 3;
			}

			/* Wait until pin is high again */
			while(!(RDATA_PORT & RDATA_BIT));
		}
		UDR0 = data_output_byte;
	}
	/* Because of the above rules the actual valid two-bit sequences output
	 * are 01, 10 and 11, so we use 00 to say "END OF DATA"
	 */
	write_byte_to_uart(0);

	/* turn off the status LED */
	LED_PORT &= ~LED_BIT;

	/* Disable the counter */
	TCCR2B = 0;	  /* No Clock (turn off) */
}

/* Read the track for a HD disk */
static void read_track_data_fast_hd(void)
{
	register unsigned char data_output_byte, counter, bits;
	long total_bits, target;

	/* Configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;			/* No physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* Prescale = 1 */

	/* First wait for the serial port to be available */
	while(!(UCSR0A & (1 << UDRE0)));

	/* Signal we're active */
	LED_PORT |= LED_BIT;

	/* While the INDEX pin is high wait if the other end requires us to */
	if(read_byte_from_uart()) while(INDEX_PORT & INDEX_BIT);

	/* Prepare the two counter values as follows: */
	TCNT2=0;	   /* Reset the counter */

	data_output_byte = 0;
	total_bits = 0;
	target = (long)RAW_HD_TRACKDATA_LENGTH * 8L;

	while(total_bits < target) {
		for(bits=0; bits<4; bits++) {
			/* Wait while pin is high */
			while(RDATA_PORT & RDATA_BIT);
			counter = TCNT2, TCNT2 = 0;  /* reset */

			data_output_byte <<= 2;

			if(counter < 40) {
				data_output_byte |= 0x01;
				total_bits += 2;
			} else if(counter > 55) {
				/* this accounts for just a '1' or a '01' as two '1' arent allowed in a row */
				data_output_byte |= 0x03;
				total_bits += 4;
			} else {
				data_output_byte |= 0x02;
				total_bits += 3;
			}

			/* Wait until pin is high again */
			while(!(RDATA_PORT & RDATA_BIT));
		}
		UDR0 = data_output_byte;
	}
	/* Because of the above rules the actual valid two-bit sequences output
	 * are 01, 10 and 11, so we use 00 to say "END OF DATA"
	 */
	write_byte_to_uart(0);

	/* turn off the status LED */
	LED_PORT &= ~LED_BIT;

	/* Disable the counter */
	TCCR2B = 0;	  /* No Clock (turn off) */
}

static char *i2a(unsigned int i, char *a, unsigned r)
{
	if (i/r>0) a=i2a(1/r,a,r);
	*a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz@$!?"[i % r];
	return a+1;
}

/* Make a statistics of the data seen on a track to determine the density of the media */
static void measure_track_data(void)
{
	// Force data to be stored in a register
	register unsigned char data_output_byte = 0, bits;
	register unsigned char counter;
	long total_bits=0;
	long target = ((long)RAW_TRACKDATA_LENGTH/4)*(long)8;
	char a[8];
	unsigned char i;

	/* for the statistics */
	int t1=0;
	int t2=0;
	int t3=0;
	int t4=0;
	int t5=0;
	int t6=0;

	/* Configure timer 2 just as a counter in NORMAL mode */
	TCCR2A = 0;			/* No physical output port pins and normal operation */
	TCCR2B = (1 << CS20);	/* Prescale = 1 */
       
	// Signal we're active
	LED_PORT |= LED_BIT;

	// While the INDEX pin is high wait 
	while(INDEX_PORT & INDEX_BIT) {};

	// Prepare the two counter values as follows:
	TCNT2=0;       // Reset the counter

	while (total_bits<target) {
		for (bits=0; bits<4; bits++) {
			/* Wait while pin is high */
			while(RDATA_PORT & RDATA_BIT) {};
			counter = TCNT2, TCNT2 = 0;  /* reset */

			data_output_byte <<= 2;

			if (counter < 40)     t1++;
			else if(counter < 55) t2++;
			else                  t3++;

			if (counter < 80)      t4++;
			else if(counter < 112) t5++;
			else                   t6++;

			total_bits += 2;

			/* Wait until pin is high again */
			while(!(RDATA_PORT & RDATA_BIT)) {};
		}
	}

	// turn off the status LED
	LED_PORT &= ~LED_BIT;

	// Disable the counter
	TCCR2B = 0;      // No Clock (turn off)

	/* Now output the result: */
	write_byte_to_uart(10);

	*i2a(t1,a,10)=0;
	i=0;
	while(a[i])
		write_byte_to_uart(a[i++]);
	write_byte_to_uart('-');

	*i2a(t2,a,10)=0;
	i=0;
	while(a[i])
		write_byte_to_uart(a[i++]);
	write_byte_to_uart('-');

	*i2a(t3,a,10)=0;
	i=0;
	while(a[i])
		write_byte_to_uart(a[i++]);
	write_byte_to_uart(10);

	*i2a(t4,a,10)=0;
	i=0;
	while(a[i])
		write_byte_to_uart(a[i++]);
	write_byte_to_uart('-');

	*i2a(t5,a,10)=0;
	i=0;
	while(a[i])
		write_byte_to_uart(a[i++]);
	write_byte_to_uart('-');

	*i2a(t6,a,10)=0;
	i=0;
	while(a[i])
		write_byte_to_uart(a[i++]);
	write_byte_to_uart(10);

	if(t5<100 && t6<100) {
	  write_byte_to_uart('H');
	  disktypeHD=1;
	} else {
	  write_byte_to_uart('D');
	  disktypeHD=0;
	}
	write_byte_to_uart('D');
	write_byte_to_uart(10);
}

/* The main command loop */
static void loop(void)
{
	unsigned char command;

	CTS_PORT &= ~CTS_BIT;		/* Allow data incoming */
	WGATE_PORT |= WGATE_BIT;   /* always turn writing off */

	/* Read the command from the PC */
	command = read_byte_from_uart();

	switch(command) {
	case '?':
		/* Command: "?" Means information about the firmware */
		write_byte_to_uart('1');  /* Success */
		write_byte_to_uart('V');
		write_byte_to_uart('1');
		write_byte_to_uart('.');
		write_byte_to_uart('7');
		break;

		/* Command "." means go back to track 0 */
	case '.':
		if(!drive_enabled) write_byte_to_uart('0'); else {
			if(goto_track0() != -1) /* reset */
				write_byte_to_uart('1');
			else write_byte_to_uart('#');
		}
		break;

	case '#':
		/* Command "#" means goto track.  Should be formatted as #00 or #32 etc */
		if(!drive_enabled) {
			read_byte_from_uart();
			read_byte_from_uart();
			write_byte_to_uart('0');
		} else
		if(goto_track_x()) {
				smalldelay(100); /* wait for drive */
				write_byte_to_uart('1');
		} else write_byte_to_uart('0');
		break;

	case '[':
		/* Command "[" select LOWER disk side */
		HEADSEL_PORT &= ~HEAD_SELECT_BIT;
		write_byte_to_uart('1');
		break;

	case ']':
		/* Command "]" select UPPER disk side */
		HEADSEL_PORT |= HEAD_SELECT_BIT;
		write_byte_to_uart('1');
		break;

	case '<':
		/* Command "<" Read track from the drive */
		if(!drive_enabled) write_byte_to_uart('0');
		else {
			write_byte_to_uart('1');
			if(disktypeHD)
				read_track_data_fast_hd();
			else
				read_track_data_fast();
		}
		break;

	case '>':
		/* Command ">" Write track to the drive */
		if(!drive_enabled) write_byte_to_uart('0'); else
		if(!in_write_mode) write_byte_to_uart('0'); else {
				write_byte_to_uart('1');
				if(disktypeHD)
					write_track_from_uart_hd();
				else
					write_track_from_uart();
		}
		break;

	case 'X':
		/* command "X" Erase current track (writes 0xAA to it) */
		if(!drive_enabled) write_byte_to_uart('0'); else
		if(!in_write_mode) write_byte_to_uart('0'); else {
				write_byte_to_uart('1');
				if(disktypeHD)
					erase_track_hd();
				else
					erase_track();
		}
		break;

	/* Command "H" Set HD disk type */
	case 'H':
		disktypeHD = 1;
		write_byte_to_uart('1');
		break;

	/* Command "D" Set DD  or SD disk type */
	case 'D':
		disktypeHD = 0;
		write_byte_to_uart('1');
		break;

	case 'M':
		if(!drive_enabled) write_byte_to_uart('0');
		else {
			write_byte_to_uart('1');
			measure_track_data();
		}
		break;

	case '-':
		/* Turn off the drive motor */
		MOTOR_PORT |= MOTOR_ENABLE_BIT;
		WGATE_PORT |= WGATE_BIT;
		drive_enabled = 0;
		write_byte_to_uart('1');
		in_write_mode = 0;
		break;

	case '+':
		/* Turn on the drive motor and setup in READ MODE */
		if(in_write_mode) {
			/* Ensure writing is turned off */
			MOTOR_PORT |= MOTOR_ENABLE_BIT;
			WGATE_PORT |= WGATE_BIT;
			smalldelay(100);
			drive_enabled = 0;
			in_write_mode = 0;
		}
		if(!drive_enabled) {
			MOTOR_PORT &= ~MOTOR_ENABLE_BIT;
			drive_enabled = 1;
			smalldelay(750); /* wait for drive */
		}
		write_byte_to_uart('1');
		break;

	case '~':
		/* Turn on the drive motor and setup in WRITE MODE */
		if(drive_enabled) {
			WGATE_PORT |= WGATE_BIT;
			MOTOR_PORT |= MOTOR_ENABLE_BIT;
			drive_enabled = 0;
			smalldelay(100);
		}
		/* We're writing! */
		WGATE_PORT &= ~WGATE_BIT;
		/* Gate has to be pulled LOW BEFORE we turn the drive on */
		MOTOR_PORT &= ~MOTOR_ENABLE_BIT;
		/* Raise the write gate again */
		WGATE_PORT |= WGATE_BIT;
		smalldelay(750); /* wait for drive */

		/* At this point we can see the status of the write protect flag */
		if((WPROT_PORT & WPROT_BIT) == 0) {
			write_byte_to_uart('0');
			in_write_mode = 0;
			MOTOR_PORT |= MOTOR_ENABLE_BIT;
			/*WGATE_PORT |= WGATE_BIT;*/
		} else  {
			in_write_mode = 1;
			drive_enabled = 1;
			write_byte_to_uart('1');
		}
		break;

	case '&':
		run_diagnostic();
		break;

	default:
		/* We don't recognise the command! */
		write_byte_to_uart('!'); /* error */
		break;
	}
}
