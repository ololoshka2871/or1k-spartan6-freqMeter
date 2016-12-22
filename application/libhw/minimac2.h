/****************************************************************************
 *
 *   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
 *   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef MINMAC_H
#define MINMAC_H

#include <stdbool.h>
#include <stdint.h>
#include "mem_map.h"

// based on https://github.com/m-labs/misoc/blob/master/misoc/software/libnet/microudp.c
//------------------------------------------------------------------------------

#define MAC_ADDR_SIZE           6

#define ETHERNET_FRAME_SIZE_MIN     64
#define ETHERNET_PAYLOAD_SIZE_MIN   (ETHERNET_FRAME_SIZE_MIN - sizeof(struct ethernet_header) - sizeof(uint32_t))

enum enMiniMACSlotStates {
    MINIMAC_SLOT_STATE_DISABLED = 0b00,
    MINIMAC_SLOT_STATE_READY = 0b01,
    MINIMAC_SLOT_STATE_DATA_RESSIVED = 0b10,
    MINIMAC_SLOT_STATE_INVALID = 0b11,
};

enum enMiniMACRxSlots {
    MINIMAC_RX_SLOT0 = 0,
    MINIMAC_RX_SLOT1 = 1,
    MINIMAC_RX_SLOT2 = 2,
    MINIMAC_RX_SLOT3 = 3,
    MINIMAC_RX_SLOT_COUNT = 4,
    MINIMAC_RX_SLOT_INVALID = 0xff
};

enum enMiniMACErrorCodes {
    MINIMAC_OK = 0,
    MINIMAC_NOMEM_ERROR = 1,
    MINIMAC_MTU_ERROR = 2,
    MINIMAC_VALUE_ERROR = 3,
    MINIMAC_SLOT_STATE_ERROR = 4,
    MINIMAC_CRC_ERROR = 5,
    MINIMAC_NO_DATA_AVALABLE = 6,
    MINIMAC_POCKET_DEST_ANOTHER = 7,
    MINIMAC_UNSUPPORTED_PROTO = 8,
    MINIMAC_HW_ERROR = 100,
};

struct ethernet_header {
    uint8_t destmac[6];
    uint8_t srcmac[6];
    uint16_t ethertype;
} __attribute__((packed));

//------------------------------------------------------------------------------

void miniMAC_init();

enum enMiniMACRxSlots miniMAC_findReadySlot();
void miniMAC_resetIfError();
uint32_t miniMAC_isReadyToTx();
uint16_t miniMAC_rxCount(enum enMiniMACRxSlots slot);
uint8_t *miniMAC_rxSlotData(enum enMiniMACRxSlots slot);
void miniMAC_acceptSlot(enum enMiniMACRxSlots slot);
void miniMAC_resetRxSlot(enum enMiniMACRxSlots slot);
uint8_t* miniMAC_txSlotData();
void miniMAC_startTransmission(uint16_t size);

#endif // MINMAC_H
