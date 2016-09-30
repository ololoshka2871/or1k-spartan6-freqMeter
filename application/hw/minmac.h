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

#ifndef MAC_CTL_BASE
#warning "MAC_CTL_BASE undefined!"
#define MAC_CTL_BASE            (FIO_BASE + 0x00100000)
#endif

#ifndef MAC_TX_MEM_BASE
#warning "MAC_TX_MEM_BASE undefined!"
#define MAC_TX_MEM_BASE         (FIO_BASE + 0x00200000)
#endif

#ifndef MAC_RX_MEM_BASE
#warning "MAC_RX_MEM_BASE undefined!"
#define MAC_RX_MEM_BASE         (FIO_BASE + 0x00300000)
#endif

#ifndef MTU
#warning "MTU undefined!"
#define MTU                     1530
#endif

//------------------------------------------------------------------------------

// Registers
#define REG_OFFSET(N)           (sizeof(uint32_t) * (N))

#define MINIMAC_RST_CTL         (*(REG32 (MAC_CTL_BASE + REG_OFFSET(0))))
#define MINIMAC_RST_RX          (1 << 0)
#define MINIMAC_RST_TX          (1 << 1)

#define MINIMAC_MDIO_BB         (*(REG32 (MAC_CTL_BASE + REG_OFFSET(1))))
#define MINIMAC_MDIO_BB_DO      (1 << 0)
#define MINIMAC_MDIO_BB_DO_0    0
#define MINIMAC_MDIO_BB_DI      (1 << 1) //RO
#define MINIMAC_MDIO_BB_DI_0    0
#define MINIMAC_MDIO_BB_OE      (1 << 2)
#define MINIMAC_MDIO_BB_OE_0    0
#define MINIMAC_MDIO_BB_CLK     (1 << 3)
#define MINIMAC_MDIO_BB_CLK_0   0

#define MINIMAC_SLOT0_STATE     (*(REG32 (MAC_CTL_BASE + REG_OFFSET(2))))
#define MINIMAC_SLOT0_ADDR      (*(REG32 (MAC_CTL_BASE + REG_OFFSET(3))))
#define MINIMAC_SLOT0_COUNT     (*(REG32 (MAC_CTL_BASE + REG_OFFSET(4)))) // RO
#define MINIMAC_SLOT_STATE(slot) (*(REG32 (MAC_CTL_BASE + REG_OFFSET(2 + (slot) * 3))))
#define MINIMAC_SLOT_ADDR(slot)  (*(REG32 (MAC_CTL_BASE + REG_OFFSET(3 + (slot) * 3))))
#define MINIMAC_SLOT_COUNT(slot) (*(REG32 (MAC_CTL_BASE + REG_OFFSET(4 + (slot) * 3))))// RO

#define MINIMAC_TX_SLOT_ADDR    (*(REG32 (MAC_CTL_BASE + REG_OFFSET(14))))
#define MINIMAC_TX_REMAINING    (*(REG32 (MAC_CTL_BASE + REG_OFFSET(15))))

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
    MINIMAC_E_NOMEM = 1,
    MINIMAC_MTU_ERROR = 2,
    MINIMAC_VALUE_ERROR = 3
};

//------------------------------------------------------------------------------

void miniMAC_control(bool rx_enable, bool tx_enable);

// interrupt handlers
void miniMAC_rx_isr(unsigned int * registers);
void miniMAC_tx_isr(unsigned int * registers);

enum enMiniMACRxSlots miniMAC_rx_static_slot_allocate();
enum enMiniMACErrorCodes miniMAC_tx_slot_allocate(uint8_t ** pslot_addr);
enum enMiniMACErrorCodes miniMAC_tx_start(uint16_t byte_count);
enum enMiniMACRxSlots miniMAC_findSlotWithState(enum enMiniMACSlotStates state);
enum enMiniMACErrorCodes miniMAC_verifyRxData(
        enum enMiniMACRxSlots slot, uint8_t** ppayload, uint16_t *ppl_size);
void miniMAC_reset_rx_slot(enum enMiniMACRxSlots slot);

uint16_t miniMAC_MDIO_ReadREG(const uint8_t phy_addr, const uint8_t reg_addr);
void miniMAC_MDIO_WriteREG(const uint8_t phy_addr, const uint8_t reg_addr,
                           const uint16_t val);
void miniMAC_MDIO_init();

#endif // MINMAC_H
