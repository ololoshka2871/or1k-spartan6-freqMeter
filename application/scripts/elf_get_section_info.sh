#!/bin/bash

ELF=$1
SEARCHREGEX=$2
FIELD=$3
STR_FROM_END=$4
READELF=$5

echo -n `${READELF} -l $ELF | grep -P "$SEARCHREGEX" | tail -n+${STR_FROM_END}\
    | head -n1 | awk -v f=$FIELD '{print $f}'`
