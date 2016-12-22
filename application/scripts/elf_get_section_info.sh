#!/bin/bash

ELF=$1
SEARCHREGEX=$2
FIELD=$3
READELF=$4

echo -n `${READELF} -l $ELF | grep -P "$SEARCHREGEX" | awk -v f=$FIELD '{print $f}'`
