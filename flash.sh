#!/bin/bash

doHelp() {
	cat <<EOF
$PROGRAM builder    
$0 [ -hlp: ]
 -h        - Help
 -l        - List ports
 -p        - Serial port to use

Running on: $OSNAME
EOF
	exit
}

gotarg=0
while getopts "hlp:" option
do
	gotarg=1
	case ${option} in
		h) doHelp
		;;
		l) ls -l /dev/cu.* 
           exit 0
        ;;
		p) PORT=$OPTARG
           echo "Using port: $PORT"
		;;
		*) doHelp
		;;
	esac
done

if [ -z "$IDF_PATH" ]; then
    echo "Please activate IDF with the export.sh script. esptool.py is needed to flash the program."
    exit 1
fi
echo "Reset ESP32 is need to start flashing..."
if [ -n "$PORT" ]; then
    esptool.py -p $PORT -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x16000 ota_data_initial.bin 0x20000 garagedoor.bin
else
    esptool.py -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x16000 ota_data_initial.bin 0x20000 garagedoor.bin
fi
