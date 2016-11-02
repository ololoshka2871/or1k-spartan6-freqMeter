#!/bin/bash

BOOTLOADER_IMAGE=$1
FPGA_DP_MEMORY_USE=$2
MEMORY_UNIT_SIZE=$3
OUTDIR=$4

BOOTLOADER_IMAGE_SIZE=`stat --printf="%s" ${BOOTLOADER_IMAGE}`
TEMPFILE=`mktemp`

function xxd_from() {
    local IN_FILE=$1
    local FROM=$2
    local SIZE=$3
    local OUT_FILE=$4

    dd if=$IN_FILE skip=${FROM} bs=1 count=$SIZE 2> /dev/null | xxd -ps -c 4 > $OUT_FILE
}

function chr() {
  [ "$1" -lt 256 ] || return 1
  printf "\\$(printf '%03o' "$1")"
}

function ord() {
  LC_CTYPE=C printf '%d' "'$1"
}

cat $BOOTLOADER_IMAGE > $TEMPFILE

zeros_needed=$((${MEMORY_UNIT_SIZE}*${FPGA_DP_MEMORY_USE}-${BOOTLOADER_IMAGE_SIZE}))

dd if=/dev/zero bs=$zeros_needed count=1 >> ${TEMPFILE} 2> /dev/null

START_V=`ord A`

OUTFILE_PATTERN=`echo $(basename ${BOOTLOADER_IMAGE}) | sed 's/\..*$/-part%s.bmm/'`

for((i=0;i<${FPGA_DP_MEMORY_USE};i++)); do
    value=$((${START_V}+${i}))
    part_file_name=`printf $OUTFILE_PATTERN $(chr ${value})`
    xxd_from ${TEMPFILE} $((${MEMORY_UNIT_SIZE}*${i})) ${MEMORY_UNIT_SIZE} \
        ${OUTDIR}/${part_file_name}
done

#rm $TEMPFILE
