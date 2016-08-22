#!/bin/bash

PORT=3333
DEVICE=/dev/ttyUSB0
BAUD=115200

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

stty -F $DEVICE -icrnl -imaxbel -opost -onlcr -isig -icanon -echo
socat -v TCP-LISTEN:${PORT},fork,reuseaddr FILE:${DEVICE},b${BAUD},raw \
	2>&1 | sed 's/\(.*\)\([<>]\) .*/\2 \1/' | $DIR/filter_debug_io_log.py
