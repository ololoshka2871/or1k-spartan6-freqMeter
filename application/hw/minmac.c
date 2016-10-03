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

#include "minmac.h"


static uint32_t align32(uint32_t x) {
    if (x & 0b11)
        return (x & 0xFFFFFFFC) + 0b100;
    return x;
}

enum enMiniMACRxSlots miniMAC_findSlotWithState(enum enMiniMACSlotStates state) {
    for (enum enMiniMACRxSlots slot = MINIMAC_RX_SLOT0; slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
        enum enMiniMACSlotStates slot_state = MINIMAC_SLOT_STATE(slot);
        if (slot_state == state)
            return slot;
    }
    return MINIMAC_RX_SLOT_INVALID;
}

void miniMAC_control(bool rx_enable, bool tx_enable) {
    MINIMAC_RST_CTL = (rx_enable ? 0 : MINIMAC_RST_RX) |
            (tx_enable ? 0 : MINIMAC_RST_TX);
}


enum enMiniMACRxSlots miniMAC_rx_static_slot_allocate() {
    // find unused slot
    enum enMiniMACRxSlots slot =
            miniMAC_findSlotWithState(MINIMAC_SLOT_STATE_DISABLED);
    if (slot == MINIMAC_RX_SLOT_INVALID)
        return slot;

    miniMAC_reset_rx_slot(slot);
    return slot;
}

enum enMiniMACErrorCodes miniMAC_tx_slot_allocate(uint8_t ** pslot_addr) {
    if (MINIMAC_TX_REMAINING != 0) {
        if (pslot_addr) *pslot_addr = NULL;
        return MINIMAC_E_NOMEM;
    }

    uint32_t* tx_slot_addr = MAC_TX_MEM_BASE;
    MINIMAC_TX_SLOT_ADDR = tx_slot_addr;
    if (pslot_addr) *pslot_addr = tx_slot_addr;
    return MINIMAC_OK;
}

enum enMiniMACErrorCodes miniMAC_tx_start(uint16_t byte_count) {
    if (byte_count <= MTU) {
        MINIMAC_TX_REMAINING = byte_count;
        return MINIMAC_OK;
    }
    return MINIMAC_MTU_ERROR;
}


// interrupt handlers
void miniMAC_rx_isrr(unsigned int * registers) {
}


void miniMAC_tx_isrr(unsigned int * registers) {
}

void miniMAC_reset_rx_slot(enum enMiniMACRxSlots slot) {
    MINIMAC_SLOT_ADDR(slot) = align32(MAC_RX_MEM_BASE + MTU * (int)slot);
    MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_READY;
}

enum enMiniMACErrorCodes miniMAC_verifyRxData(enum enMiniMACRxSlots slot,
        uint8_t **ppayload, uint16_t *ppl_size) {
    enum enMiniMACErrorCodes ret = MINIMAC_OK;

    // block slot
    MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_DISABLED;

    // verifying
    // if ()
    // if ()
    // ...
    if (ppayload && ppl_size) {
        // TODO
        *ppayload = MINIMAC_SLOT_ADDR(slot);
        *ppl_size = MINIMAC_SLOT_COUNT(slot);
    }
    return ret; // OK

__verify_error:
    if (ppayload && ppl_size) {
        // TODO
        *ppayload = NULL;
        *ppl_size = 0;
    }
    return ret; // error code there
}

