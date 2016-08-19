#!/bin/bash

PRJ_TMP_FILE=$1
PRJ_FILE=$2

if [[ -f $PRJ_FILE ]]; then
	PRJ_FILE_DATE=`date -d "$(stat -c %y $PRJ_FILE)" +%s`
else
	PRJ_FILE_DATE=0
fi

PRJ_TMP_FILE_DATE=`date -d "$(stat -c %y $PRJ_TMP_FILE)" +%s`

if [[ "$PRJ_TMP_FILE_DATE" > "$PRJ_FILE_DATE" ]]; then
        echo "PRJ template $PRJ_TMP_FILE changed"
	cp $PRJ_TMP_FILE $PRJ_FILE
	exit 0
fi

while read line; do
	FILE=`echo "$line" | awk '{print $3}'`
	FILE_DATE=`date -d "$(stat -c %y $FILE)" +%s`
        #echo "Checking $FILE ($FILE_DATE) ($PRJ_FILE_DATE)"
        if [[ "$FILE_DATE" > "$PRJ_FILE_DATE" ]]; then
                echo "Source file $FILE changed"
		cp $PRJ_TMP_FILE $PRJ_FILE
		exit 0
	fi
done < $PRJ_TMP_FILE
