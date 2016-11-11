#!/bin/bash

FIRST_BYTE=$1

echo "#ifndef __GENERATED_MAC_H_"
echo "#define __GENERATED_MAC_H_"
echo

date | md5sum | sed "s/^\(..\)\(..\)\(..\)\(..\)\(..\).*$/\#define ETH_MAC0 (0x${FIRST_BYTE})\n\#define ETH_MAC1 (0x\1)\n\#define ETH_MAC2 (0x\2)\n\#define ETH_MAC3 (0x\3)\n\#define ETH_MAC4 (0x\4)\n\#define ETH_MAC5 (0x\5)\n/"

echo "#endif /*__GENERATED_MAC_H_*/"
