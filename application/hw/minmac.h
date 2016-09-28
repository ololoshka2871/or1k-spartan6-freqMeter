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
#include "mem_map.h"

#ifndef MAC_TX_MEM_BASE
#warning "MAC_TX_MEM_BASE undefined!"
#define MAC_TX_MEM_BASE         0x11200000
#endif

#ifndef MAC_RX_MEM_BASE
#warning "MAC_RX_MEM_BASE undefined!"
#define MAC_RX_MEM_BASE         0x11300000
#endif

#ifndef MAC_CTL_BASE
#warning "MAC_CTL_BASE undefined!"
#define MAC_CTL_BASE            0x11100000
#endif

#ifndef MTU
#warning "MTU undefined!"
#define MTU                     1530
#endif

//------------------------------------------------------------------------------

// Registers

#define MINMAC_RST_CTL          (*(REG32 (MAC_CTL_BASE + 0x0)))
#define MINMAC_RST_RX           (1 << 0)
#define MINMAC_RST_TX           (1 << 1)

#define MINMAC_MDIO_BB          (*(REG32 (MAC_CTL_BASE + 0x4)))
#define MINMAC_MDIO_BB_DO       (1 << 0)
#define MINMAC_MDIO_BB_DI       (1 << 1) //RO
#define MINMAC_MDIO_BB_OE       (1 << 2)
#define MINMAC_MDIO_BB_CLK      (1 << 3)

#define MINMAC_SLOT0_STATE      (*(REG32 (MAC_CTL_BASE + 0x8)))
#define MINMAC_SLOT0_ADDR       (*(REG32 (MAC_CTL_BASE + 0xb)))
#define MINMAC_SLOT1_STATE      (*(REG32 (MAC_CTL_BASE + 0x10)))
#define MINMAC_SLOT1_ADDR       (*(REG32 (MAC_CTL_BASE + 0x14)))
#define MINMAC_SLOT2_STATE      (*(REG32 (MAC_CTL_BASE + 0x18)))
#define MINMAC_SLOT2_ADDR       (*(REG32 (MAC_CTL_BASE + 0x1b)))
#define MINMAC_SLOT3_STATE      (*(REG32 (MAC_CTL_BASE + 0x20)))
#define MINMAC_SLOT4_ADDR       (*(REG32 (MAC_CTL_BASE + 0x24)))
#define MINMAC_TX_ADDR          (*(REG32 (MAC_CTL_BASE + 0x20)))
#define MINMAC_TX_REMAINING     (*(REG32 (MAC_CTL_BASE + 0x24)))

#define MINMAC_SLOT_STATE(slot) (*(REG32 (MAC_CTL_BASE + 0x10 + (slot) * 8)))
#define MINMAC_SLOT_ADDR(slot)  (*(REG32 (MAC_CTL_BASE + 0x14 + (slot) * 8)))

enum enMinmacSlotStates {
    MINMAC_SLOT_STATE_DISABLED = 0b00,
    MINMAC_SLOT_STATE_READY = 0b01,
    MINMAC_SLOT_STATE_DATA_RESSIVED = 0b10,
    MINMAC_SLOT_STATE_INVALID = 0b11,
};

enum enMinmacRxSlots {
    MINMAC_RX_SLOT0 = 0,
    MINMAC_RX_SLOT1 = 1,
    MINMAC_RX_SLOT2 = 2,
    MINMAC_RX_SLOT3 = 3,
    MINMAC_RX_SLOT_COUNT = 4,
    MINMAC_RX_SLOT_INVALID = 0xff
};

enum enMinmacErrorCodes {
    MINMAC_OK = 0,
    MINMAC_E_NOMEM = 1,
};

//------------------------------------------------------------------------------

void minmac_control(bool rx_enable, bool tx_enable);

// interrupt handlers
void minmac_rx_isr(unsigned int * registers);
void minmac_tx_isr(unsigned int * registers);

enum enMinmacRxSlots minmac_rx_static_slot_alocate();


#endif // MINMAC_H
