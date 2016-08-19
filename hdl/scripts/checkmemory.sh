#!/bin/bash

MEM_TMP_FILE=$1
MEM_FILE=$2
HEX_IMAGE=$3


if [[ -f $MEM_FILE ]]; then
	MEM_FILE_DATE=`date -d "$(stat -c %y $MEM_FILE)" +%s`
else
	MEM_FILE_DATE=0
fi

MEM_TMP_FILE_DATE=`date -d "$(stat -c %y $MEM_TMP_FILE)" +%s`

IMAGE_DATE=`date -d "$(stat -c %y $HEX_IMAGE)" +%s`

if [[ "$IMAGE_DATE" > "$MEM_FILE_DATE" ]]; then
        echo "Memory image $HEX_IMAGE changed"
	cp $MEM_TMP_FILE $MEM_FILE
	exit 0
fi

if [[ "$MEM_TMP_FILE_DATE" > "$MEM_FILE_DATE" ]]; then
        echo "Memory image $MEM_TMP_FILE_DATE changed"
	cp $MEM_TMP_FILE $MEM_FILE
	exit 0
fi
