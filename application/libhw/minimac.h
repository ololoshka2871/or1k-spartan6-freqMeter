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

#define ETHERTYPE_ARP           0x0806
#define ETHERTYPE_IP            0x0800

#define ETHERNET_FRAME_SIZE_MIN 64

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
    MINIMAC_NO_DATA_AVALABLE = 6
};

struct sminiMAC_Stat {
    uint32_t pocket_rx;
    uint32_t pocket_rx_errors;

    uint32_t pocket_tx;
};

struct ethernet_header {
    uint8_t destmac[6];
    uint8_t srcmac[6];
    uint16_t ethertype;
} __attribute__((packed));


union uethernet_buffer {
    struct ethernet_header frame;
    unsigned char raw[1532];
};

//------------------------------------------------------------------------------

void miniMAC_control(bool rx_enable, bool tx_enable);

enum enMiniMACRxSlots miniMAC_rx_static_slot_allocate();
uint8_t* miniMAC_tx_slot_allocate(size_t wanted_size);
uint8_t* miniMAC_slot_prepare(const uint8_t dest_mac[],
                              uint16_t ether_type, uint8_t* slot);
void miniMAC_slot_complite_and_send(uint8_t* slot_data);
enum enMiniMACErrorCodes miniMAC_tx_start(uint16_t byte_count);
enum enMiniMACRxSlots miniMAC_findSlotWithState(enum enMiniMACSlotStates state);
enum enMiniMACErrorCodes miniMAC_getpointerRxDatarRxData(
        enum enMiniMACRxSlots *pslot, union uethernet_buffer** ppayload,
        uint16_t *ppl_size);
void miniMAC_reset_rx_slot(enum enMiniMACRxSlots slot);
enum enMiniMACRxSlots miniMAC_is_data_ressived();
void minMAC_stat(struct sminiMAC_Stat* pstst);

extern uint8_t myMAC[6];
extern const uint8_t broadcastMAC[6];

#endif // MINMAC_H
