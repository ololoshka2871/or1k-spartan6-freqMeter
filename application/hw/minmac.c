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

#include "minmac.h"

static enum enMinmacRxSlots findSlotWithState(enum enMinmacSlotStates state) {
    for (enum enMinmacRxSlots slot = 0; slot < MINMAC_RX_SLOT_COUNT; ++slot) {
        enum enMinmacSlotStates slot_state = MINMAC_SLOT_STATE(slot);
        if (slot_state == state)
            return slot;
    }
    return MINMAC_RX_SLOT_INVALID;
}

void minmac_control(bool rx_enable, bool tx_enable) {
    MINMAC_RST_CTL = (uint32_t)rx_enable | (((uint32_t)tx_enable) << 1);
}


enum enMinmacRxSlots minmac_rx_static_slot_alocate() {
    // find unused slot
    enum enMinmacRxSlots slot = findSlotWithState(MINMAC_SLOT_STATE_DISABLED);
    if (slot == MINMAC_RX_SLOT_INVALID)
        return slot;

    static const uint32_t static_rx_alocation_table[] = {
        MAC_RX_MEM_BASE + 0,
        MAC_RX_MEM_BASE + MTU * 1,
        MAC_RX_MEM_BASE + MTU * 2,
        MAC_RX_MEM_BASE + MTU * 3
    };
    MINMAC_SLOT_ADDR(slot) = static_rx_alocation_table[slot];
    MINMAC_SLOT_STATE(slot) = MINMAC_SLOT_STATE_READY;
    return slot;
}


// interrupt handlers
void minmac_rx_isr(unsigned int * registers) {
}


void minmac_tx_isr(unsigned int * registers) {
}
