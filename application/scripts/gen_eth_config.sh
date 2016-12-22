#!/bin/bash

FIRST_BYTE=$1

STATIC_IP=$2
STATIC_NETMASK=$3
STATIC_GW=$4

is_forced_MAC=`echo $FIRST_BYTE | grep ':'`

echo "#ifndef __GENERATED_ETH_CONFIG_H_"
echo "#define __GENERATED_ETH_CONFIG_H_"
echo

if [[ -z $is_forced_MAC ]]; then
    # generated
    date | md5sum | sed "s/^\(..\)\(..\)\(..\)\(..\)\(..\).*$/\#define ETH_MAC0 (0x${FIRST_BYTE})\n\#define ETH_MAC1 (0x\1)\n\#define ETH_MAC2 (0x\2)\n\#define ETH_MAC3 (0x\3)\n\#define ETH_MAC4 (0x\4)\n\#define ETH_MAC5 (0x\5)\n/"
else
    # forced
    echo $FIRST_BYTE | sed "s/^\(..\):\(..\):\(..\):\(..\):\(..\):\(..\)*$/\#define ETH_MAC0 (0x\1)\n\#define ETH_MAC1 (0x\2)\n\#define ETH_MAC2 (0x\3)\n\#define ETH_MAC3 (0x\4)\n\#define ETH_MAC4 (0x\5)\n\#define ETH_MAC5 (0x\6)\n/"
fi

echo $STATIC_IP      | sed "s/^\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\).*$/\#define ETH_IP0 (\1)\n\#define ETH_IP1 (\2)\n\#define ETH_IP2 (\3)\n\#define ETH_IP3 (\4)\n/"

echo $STATIC_NETMASK | sed "s/^\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\).*$/\#define ETH_NETMASK0 (\1)\n\#define ETH_NETMASK1 (\2)\n\#define ETH_NETMASK2 (\3)\n\#define ETH_NETMASK3 (\4)\n/"

echo $STATIC_GW      | sed "s/^\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\)\.\([0-9]*\).*$/\#define ETH_GW0 (\1)\n\#define ETH_GW1 (\2)\n\#define ETH_GW2 (\3)\n\#define ETH_GW3 (\4)\n/"

echo "#endif /*__GENERATED_ETH_CONFIG_H_*/"
