/*
amigafloppy - driver for the USB floppy controller for amiga disks
Copyright (C) 2018  John Tsiombikas <nuclear@member.fsf.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include "serial.h"

static int baud_id(int baud);

int ser_open(const char *port, int baud, unsigned int mode)
{
	int fd;
	struct termios term;

	if((baud = baud_id(baud)) == -1) {
		fprintf(stderr, "ser_open: invalid baud number: %d\n", baud);
		return -1;
	}

	if((fd = open(port, O_RDWR | O_NOCTTY)) == -1) {
		fprintf(stderr, "ser_open: failed to open device: %s: %s\n", port, strerror(errno));
		return -1;
	}

	tcgetattr(fd, &term);

	term.c_oflag = 0;
	term.c_lflag = 0;
	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME] = 0;

	term.c_cflag = CLOCAL | CREAD | CS8 | HUPCL;
	if(mode & SER_8N2) {
		term.c_cflag |= CSTOPB;
	}
	if(mode & SER_HWFLOW) {
		term.c_cflag |= CRTSCTS;
	}

	term.c_iflag = IGNBRK | IGNPAR;

	cfsetispeed(&term, baud);
	cfsetospeed(&term, baud);

	if(tcsetattr(fd, TCSANOW, &term) < 0) {
		fprintf(stderr, "ser_open: failed to set terminal attributes\n");
		close(fd);
		return -1;
	}

#ifdef TIOCM_RTS
	/* assert DTR/RTS lines */
	{
		int st;
		if(ioctl(fd, TIOCMGET, &st) == -1) {
			perror("ser_open: failed to get modem status");
			close(fd);
			return -1;
		}
		st |= TIOCM_DTR | TIOCM_RTS;
		if(ioctl(fd, TIOCMSET, &st) == -1) {
			perror("ser_open: failed to set flow control");
			close(fd);
			return -1;
		}
	}
#endif

	return fd;
}

void ser_close(int fd)
{
	close(fd);
}

int ser_block(int fd)
{
	return fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
}

int ser_nonblock(int fd)
{
	return fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
}

int ser_pending(int fd)
{
	static struct timeval tv_zero;
	fd_set rd;

	FD_ZERO(&rd);
	FD_SET(fd, &rd);

	while(select(fd + 1, &rd, 0, 0, &tv_zero) == -1 && errno == EINTR);
	return FD_ISSET(fd, &rd);
}

int ser_wait(int fd, long msec)
{
	struct timeval tv, tv0;
	fd_set rd;

	FD_ZERO(&rd);
	FD_SET(fd, &rd);

	tv.tv_sec = msec / 1000;
	tv.tv_usec = msec * 1000;

	gettimeofday(&tv0, 0);

	while(select(fd + 1, &rd, 0, 0, msec >= 0 ? &tv : 0) == -1 && errno == EINTR) {
		/* interrupted, recalc timeout and go back to sleep */
		if(msec >= 0) {
			gettimeofday(&tv, 0);
			msec -= (tv.tv_sec - tv0.tv_sec) * 1000 + (tv.tv_usec - tv0.tv_usec) / 1000;
			if(msec < 0) msec = 0;

			tv.tv_sec = msec / 1000;
			tv.tv_usec = msec * 1000;
		}
	}

	return FD_ISSET(fd, &rd);
}

int ser_write(int fd, const void *buf, int count)
{
	int i;
	unsigned char *ptr = (unsigned char *) buf;
	return write(fd, buf, count);
}

int ser_read(int fd, void *buf, int count)
{
	int i, retval;
	unsigned char *ptr = (unsigned char *) buf;
	retval = read(fd, buf, count);
	return rval;
}

void ser_printf(int fd, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vdprintf(fd, fmt, ap);
	va_end(ap);
}

char *ser_getline(int fd, char *buf, int bsz)
{
	static char linebuf[512];
	static int widx;
	int i, rd, size, offs;

	size = sizeof linebuf - widx;
	while(size && (rd = read(fd, linebuf + widx, size)) > 0) {
		widx += rd;
		size -= rd;
	}

	linebuf[widx] = 0;

	for(i=0; i<widx; i++) {
		if(linebuf[i] == '\r' || linebuf[i] == '\n') {
			size = i >= bsz ? bsz - 1 : i;
			memcpy(buf, linebuf, size);
			buf[size] = 0;

			offs = i + 1;
			memmove(linebuf, linebuf + offs, widx - offs);
			widx -= offs;
			return buf;
		}
	}
	return 0;
}

static int baud_id(int baud)
{
	switch(baud) {
	case 50: return B50;
	case 75: return B75;
	case 110: return B110;
	case 134: return B134;
	case 150: return B150;
	case 200: return B200;
	case 300: return B300;
	case 600: return B600;
	case 1200: return B1200;
	case 1800: return B1800;
	case 2400: return B2400;
	case 4800: return B4800;
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	case 1000000: return B1000000;
	case 2000000: return B2000000;
	default:
		break;
	}
	return -1;
}

void ser_flush(int fd)
{
	if (fd >= 0)
		tcflush(fd, TCIFLUSH);

	if (fd >= 0)
		tcflush(fd, TCOFLUSH);
}
