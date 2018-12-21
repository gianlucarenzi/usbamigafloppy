#!/bin/bash
#
# $Id: jtag-key-flasher.sh,v 1.1 2018/12/20 15:38:45 gianluca Exp $
#
FTDI=`which ftdi_eeprom`
ANSI="~/Progetti/iMX28/scripts/ansi.sh"
ORIG_CONFIG="jtagkey.cfg"
TEMP_CONFIG="/tmp/jtagkey.cfg"
WGET=`which wget`
WPUT=`which wput`
DATABASENAME="waffle-serial.db"
SERIAL="/tmp/$DATABASENAME"
SUDO="sudo"

# Clear screen
reset
clear

# Ansi-colored
source ~/Progetti/iMX28/scripts/ansi.sh

function check_software()
{
	app=$1
	if [ ! -f $app ]
	then
		error "Need $app. Please install"
		exit 1
	fi
}

# Controlliamo la presenza nel sistema delle utility necessarie
# ad una corretta funzionalita` dell'applicativo

check_software $FTDI
check_software $WGET
check_software $WPUT

if [ ! -f $SERIAL ]
then
	warning "Download waffle-serial.db from server..."
	$WGET ftp://gianluca:ol61renzi@server.eptar.com/software/jtagkey/$DATABASENAME -O $SERIAL
	if [ $? -ne 0 ]
	then
		error "Unable to download file"
		exit 1
	fi
fi

warning "Creating new serial number"
ncols=0
while [ true ];
do
	# Prendiamo un numero seriale random
	NEWSERIAL=`cat /dev/urandom | tr -dc 'A-Z0-9' | fold -w 6 | head -n 1`
	FOUND=`grep $NEWSERIAL $SERIAL`
	if [ "$FOUND" == "" ]; then
		echo
		break
	else
		echo -n "."
		# Al massimo facciamo 80 caratteri per riga, dopo c'e` un CR/LF
		ncols=$(($ncols+1))
		if [ $ncols -gt 80 ]
		then
			ncols=0
			echo
		fi
	fi
done

# Creiamo una configurazione speciale (per il serial number)
cp $ORIG_CONFIG $TEMP_CONFIG

# Aggiorniamo il file di configurazione con il nuovo seriale
sed -i 's/DEADBEEF/'"WAFFLE-$NEWSERIAL"'/g' $TEMP_CONFIG
sed -i 's/6010/6010/g' $TEMP_CONFIG

warning "#################################"
warning "New serial number will be: $NEWSERIAL"
warning "#################################"

# Per prima cosa cancelliamo la eeprom
# N.B.: il file di configurazione e` ridondante
alert "FTDI EEPROM ERASE..."
$SUDO $FTDI --erase-eeprom $TEMP_CONFIG
if [ $? -ne 0 ]
then
	# Proviamo a cancellare una schedina GIA` programmata
	sed -i 's/6010/6010/g' $TEMP_CONFIG
	$SUDO $FTDI --erase-eeprom $TEMP_CONFIG
	if [ $? -ne 0 ]
	then
		error "Unable to erase eeprom"
		rm $TEMP_CONFIG
		rm $SERIAL
		exit 1
	else
		warning "Reconnect the RBL003 WAFFLE please, and re-run this script"
		exit 0
	fi
fi
# Ricommutiamo con la modalita` jtagkey
sed -i 's/6010/6010/g' $TEMP_CONFIG
alert "FTDI EEPROM FLASH..."
$SUDO $FTDI --flash-eeprom $TEMP_CONFIG
if [ $? -ne 0 ]
then
	error "Unable to flash eeprom"
	rm $TEMP_CONFIG
	rm $SERIAL
	exit 1
fi

# Aggiorniamo il database dei serial number nella cartella software/jtag
# dell'utente gianluca
echo $NEWSERIAL >> $SERIAL
warning "Updating jtagkey-serial.db on server with $NEW_SERIAL"
wput --basename=$SERIAL $SERIAL ftp://gianluca:ol61renzi@server.eptar.com/software/jtagkey/$DATABASENAME
if [ $? -ne 0 ]
then
	error "Unable to upload. The $NEW_SERIAL serial will be freed"
	rm $TEMP_CONFIG
	rm $SERIAL
	exit 1
fi

warning "Remove and attach board"
while [ true ]
do
	RM=`lsusb | grep '0403'`
	if [ "$RM" != "" ]
	then
		break
	fi
	sleep 1
done

warning "Checking for correct serial number"
while [ true ]
do
	INSTALLED=`lsusb -vv |grep iSerial |awk '{print $3}' |grep WAFFLE`
	if [ "$INSTALLED" != "" ]
	then
		break
	fi
	sleep 1
done

good "Device WAFFLE $INSTALLED correctly installed"
exit 0
