#!/bin/bash

LD_SCRIPT_TEMPLATE=$1
BOOTLOADER_ELF=$2
HEADER_W1=$3
HEADER_W2=$4
USER_CODE_FLASH_OFFSET=$5
MAC_TX_MEM_BASE=$6
MAC_RX_MEM_BASE=$7
MYMINMAC_RX_SLOTS=$8
MYMINMAC_TX_SLOTS=$9
MEMORY_UNIT_SIZE=${10}
MTU=${11}
TOOLCHAIN_PREFIX=${12}
PYTHON_EXECUTABLE=${13}

if [[ $# -ne 13 ]]; then
    echo "Incorrect call. Only for internal use."
    exit 1
fi

get_section_info="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/elf_get_section_info.sh

BOOTLOADER_START=`$get_section_info $BOOTLOADER_ELF "LOAD.*RWE" 3 ${TOOLCHAIN_PREFIX}readelf`
BOOTLOADER_SIZE=`$get_section_info $BOOTLOADER_ELF "LOAD.*RWE" 6 ${TOOLCHAIN_PREFIX}readelf`
BOOTLOADER_END=`$get_section_info $BOOTLOADER_ELF "LOAD.*RW " 3 ${TOOLCHAIN_PREFIX}readelf`

APP_START="$BOOTLOADER_START + $BOOTLOADER_SIZE + 8"
APP_SIZE="$BOOTLOADER_END - ($APP_START)"

MAC_TX_MEM_SIZE=`$PYTHON_EXECUTABLE -c "import math; print(int(\
    $MEMORY_UNIT_SIZE / 8 * int(math.ceil($MTU.0 * $MYMINMAC_TX_SLOTS / ($MEMORY_UNIT_SIZE / 8)))))"`
MAC_RX_MEM_SIZE=`$PYTHON_EXECUTABLE -c "import math; print(int(\
    $MEMORY_UNIT_SIZE / 8 * int(math.ceil($MTU.0 * $MYMINMAC_RX_SLOTS / ($MEMORY_UNIT_SIZE / 8)))))"`

sed -e "s/@APP_START@/$APP_START/"\
    -e "s/@APP_SIZE@/$APP_SIZE/"\
    -e "s/@HEADER_W1@/$HEADER_W1/"\
    -e "s/@HEADER_W2@/$HEADER_W2/"\
    -e "s/@FLASH_TEXT_START@/$USER_CODE_FLASH_OFFSET/"\
    -e "s/@MAC_TX_MEM_BASE@/$MAC_TX_MEM_BASE/"\
    -e "s/@MAC_RX_MEM_BASE@/$MAC_RX_MEM_BASE/"\
    -e "s/@MAC_TX_MEM_SIZE@/$MAC_TX_MEM_SIZE/"\
    -e "s/@MAC_RX_MEM_SIZE@/$MAC_RX_MEM_SIZE/"\
    $LD_SCRIPT_TEMPLATE
