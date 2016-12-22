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

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include "minimac2.h"

#ifndef MAC_CTL_BASE
#warning "MAC_CTL_BASE undefined!"
#define MAC_CTL_BASE             (FIO_BASE + 0x00100000)
#endif

#ifndef MAC_TX_MEM_BASE
#warning "MAC_TX_MEM_BASE undefined!"
#define MAC_TX_MEM_BASE          (FIO_BASE + 0x00200000)
#endif

#ifndef MAC_RX_MEM_BASE
#warning "MAC_RX_MEM_BASE undefined!"
#define MAC_RX_MEM_BASE          (FIO_BASE + 0x00300000)
#endif

#ifndef MTU
#warning "MTU undefined!"
#define MTU                      1530
#endif


// Registers
#define REG_OFFSET(N)            ((N) << 2)

#define STATE_OFFSET             REG_OFFSET(0)
#define COUNT_OFFSET             REG_OFFSET(4)

#define MINIMAC_RST_CTL          (*(REG32 (MAC_CTL_BASE + REG_OFFSET(15))))
#define MINIMAC_RST_RX           (1 << 0)
#define MINIMAC_RST_TX           (1 << 1)

#define MINIMAC_SLOT_STATE(slot) (*(REG32 (MAC_CTL_BASE + STATE_OFFSET + REG_OFFSET(slot))))
#define MINIMAC_SLOT_COUNT(slot) (*(REG32 (MAC_CTL_BASE + COUNT_OFFSET + REG_OFFSET(slot))))  // RO

#define MINIMAC_TX_REMAINING     (*(REG32 (MAC_CTL_BASE + REG_OFFSET(14))))

#define MINIMAC_ENABLE_RX(enable) \
    do {\
        if (enable) \
            MINIMAC_RST_CTL &= ~MINIMAC_RST_RX;\
        else \
            MINIMAC_RST_CTL |= MINIMAC_RST_RX;\
    } while (0)

#define MINIMAC_ENABLE_TX(enable) \
    do {\
        if (enable) \
            MINIMAC_RST_CTL &= ~MINIMAC_RST_TX;\
        else \
            MINIMAC_RST_CTL |= MINIMAC_RST_TX;\
    } while (0)


static uint32_t Rx_slots[MINIMAC_RX_SLOT_COUNT];

static inline uint32_t align32(uint32_t v) {
    return (v & 0b11) ? ((v & ~0b11) + 0b100) : v;
}

static void assert_slot(enum enMiniMACRxSlots slot) {
    assert(slot < MINIMAC_RX_SLOT_COUNT);
}

void miniMAC_init() {
    MINIMAC_ENABLE_RX(true);
    MINIMAC_ENABLE_TX(true);

    for(enum enMiniMACRxSlots slot = MINIMAC_RX_SLOT0;
             slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
        Rx_slots[slot] = ((uint32_t)MAC_RX_MEM_BASE + align32((uint32_t)MTU * slot));
        MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_READY;
    }
}

enum enMiniMACRxSlots miniMAC_findReadySlot() {
    for (enum enMiniMACRxSlots slot = MINIMAC_RX_SLOT0;
        slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
        enum enMiniMACSlotStates slot_state = MINIMAC_SLOT_STATE(slot);
        if (slot_state == MINIMAC_SLOT_STATE_DATA_RESSIVED)
            return slot;
    }

    return MINIMAC_RX_SLOT_INVALID;
}

void miniMAC_acceptSlot(enum enMiniMACRxSlots slot) {
    assert_slot(slot);

    MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_DISABLED;
}

void miniMAC_resetIfError() {
    if (MINIMAC_RST_CTL & MINIMAC_RST_RX) {
        // ressive error
        // слот, который сейчас активен вызвал ошибку приёма, сбросим его
        // затем разрешим работу приёмника
        for (enum enMiniMACRxSlots slot = MINIMAC_RX_SLOT0;
             slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
            uint32_t count = MINIMAC_SLOT_COUNT(slot);
            enum enMiniMACSlotStates slot_state = MINIMAC_SLOT_STATE(slot);
            if (count && (slot_state == MINIMAC_SLOT_STATE_READY))
                miniMAC_resetRxSlot(slot);
        }
        MINIMAC_ENABLE_RX(true);
    }
}

uint32_t miniMAC_isReadyToTx() {
    return !MINIMAC_TX_REMAINING;
}

uint16_t miniMAC_rxCount(enum enMiniMACRxSlots slot) {
    assert_slot(slot);

    return MINIMAC_SLOT_COUNT(slot);
}

uint8_t *miniMAC_rxSlotData(enum enMiniMACRxSlots slot) {
    assert_slot(slot);

    return (uint8_t*)Rx_slots[slot];
}

void miniMAC_resetRxSlot(enum enMiniMACRxSlots slot) {
    assert_slot(slot);

    MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_READY;
}

uint8_t* miniMAC_txSlotData() {
    return (uint8_t*)MAC_TX_MEM_BASE;
}

void miniMAC_startTransmission(uint16_t size) {
    MINIMAC_TX_REMAINING = size;
}
