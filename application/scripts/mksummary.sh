#!/bin/bash

BOOTLOADER_IMAGE_ELF=$1
APPLICATION_ELF_NAME=$2
TOOLCHAIN_PREFIX=$3
SUMMARY_FILE_NAME=$4
PYTHON_EXECUTABLE=$5

function elf_get_symbol_addr() {
    local ELF=$1
    local SYMBOL=$2

    echo -n `${TOOLCHAIN_PREFIX}readelf -s $ELF | grep $SYMBOL | awk '{print $2}'`
}

function calc_len() {
    echo -n `$PYTHON_EXECUTABLE -c "print('0x{:08X}'.format($1 - $2))"`
}

function calc_len_0x() {
    local p1="0x${1}"
    local p2="0x${2}"
    echo -n `$PYTHON_EXECUTABLE -c "print('0x{:08X}'.format($p1 - $p2))"`
}

function print_str() {
    local SECTION_NAME_SIZE=20
    local VALUE_SIZE=12
    local PRC_SIZE=7

    printf "%-${SECTION_NAME_SIZE}s | %${VALUE_SIZE}s | %${VALUE_SIZE}s | %${PRC_SIZE}s\n" \
        $@ >> ${SUMMARY_FILE_NAME}
}

function calc_percentage() {
    PEACE=$1
    FULL=$2

    echo -n `$PYTHON_EXECUTABLE -c "print('{:0.2f}%'.format(float($PEACE)/$FULL*100))"`
}

function section_summary() {
    local ELF=$1
    local SECTION_NAME=$2
    local START_LABEL=$3
    local END_LABEL=$4
    local TOTAM_MEM_SIZE=$5
    local USED=$6

    local section_start=`elf_get_symbol_addr $ELF $START_LABEL`
    local section_end=`elf_get_symbol_addr $ELF $END_LABEL`
    local section_size=`calc_len_0x $section_end $section_start`
    local section_usage=`calc_percentage ${section_size} ${TOTAM_MEM_SIZE}`

    print_str "${SECTION_NAME}" 0x${section_start} ${section_size} ${section_usage}

    echo -n `$PYTHON_EXECUTABLE -c "print('0x{:08X}'.format(${USED} + ${section_size}))"`
}

get_section_info="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/elf_get_section_info.sh

echo "####################### Firmware summary ####################" > ${SUMMARY_FILE_NAME}
print_str "Section" "START" "SIZE" "USAGE"
print_str "====================" "============" "============" "========"

# get bootloader elf summary
SYS_MEMORY_START=`$get_section_info $BOOTLOADER_IMAGE_ELF "LOAD.*RWE" 3 ${TOOLCHAIN_PREFIX}readelf`
BL_TEXT_SIZE=`$get_section_info $BOOTLOADER_IMAGE_ELF "LOAD.*RWE" 6 ${TOOLCHAIN_PREFIX}readelf`
SYS_MEMORY_END=`$get_section_info $BOOTLOADER_IMAGE_ELF "LOAD.*RW " 3 ${TOOLCHAIN_PREFIX}readelf`

SYS_MEMORY_SIZE=`calc_len ${SYS_MEMORY_END} ${SYS_MEMORY_START}`
BL_TEXT_USAGE=`calc_percentage ${BL_TEXT_SIZE} ${SYS_MEMORY_SIZE}`

print_str "bootloader.text" ${SYS_MEMORY_START} ${BL_TEXT_SIZE} ${BL_TEXT_USAGE}

TOTAL_USED=$BL_TEXT_SIZE

TOTAL_USED=`section_summary $BOOTLOADER_IMAGE_ELF "bootloader.bss" \
    "_bss_start" "_bss_end" ${SYS_MEMORY_SIZE} ${TOTAL_USED}`
TOTAL_USED=`section_summary $BOOTLOADER_IMAGE_ELF "bootloader.heap" \
    "__heap_start__" "__heap_end__" ${SYS_MEMORY_SIZE} ${TOTAL_USED}`

print_str "--------------------" "------------" "------------" "--------"

TOTAL_USED=`section_summary $APPLICATION_ELF_NAME "application.text" \
    "hader_start" "_text_end" ${SYS_MEMORY_SIZE} ${TOTAL_USED}`
TOTAL_USED=`section_summary $APPLICATION_ELF_NAME "application.bss" \
    "_bss_start" "_bss_end" ${SYS_MEMORY_SIZE} ${TOTAL_USED}`
TOTAL_USED=`section_summary $APPLICATION_ELF_NAME "application.heap" \
    "__heap_start__" "__heap_end__" ${SYS_MEMORY_SIZE} ${TOTAL_USED}`

print_str "--------------------" "------------" "------------" "--------"

# stack info
STACK_SIZE=`$PYTHON_EXECUTABLE -c "print('0x{:08X}'.format(${SYS_MEMORY_SIZE} - $TOTAL_USED))"`
STACK_USAGE=`calc_percentage ${STACK_SIZE} ${SYS_MEMORY_SIZE}`

print_str "Stack" ${SYS_MEMORY_END} ${STACK_SIZE} ${STACK_USAGE}

echo "#############################################################" >> ${SUMMARY_FILE_NAME}

cat $SUMMARY_FILE_NAME
