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

#include <stddef.h>
#include <string.h>

#include "gdb-stub-sections.h"

#include "freqmeters.h"
#include "minimac.h"
#include "mdio.h"

static void GDB_STUB_SECTION_TEXT test_freqmeter() {
    fm_init();

    FM_IF = 0xFFFFFFFF;
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        FM_START_VAL_CH(i);
        FM_STOP_VAL_CH(i);
        fm_setChanelReloadValue(i, 10, false);
        fm_enableChanel(i, true);
    }
    // sim to 280us
}

static void GDB_STUB_SECTION_TEXT test_multiplication() {
    volatile uint32_t a = 15;
    volatile uint32_t b = 48;
    volatile uint32_t res;

    asm volatile("l.mul %0, %1, %2" : "=r" (res) : "r" (a), "r" (b)); // sim to 130us
}

static void GDB_STUB_SECTION_TEXT test_mdio() {
    // Test MDIO
    for(uint8_t i = 0; i < 5; ++i) {
        MDIO_WriteREG(i, PHY_ANNPTR - i, PHY_BMCR_SPEED100MB | PHY_BMCR_FULL_DUPLEX);
        MDIO_ReadREG_sync(i, PHY_ANNPTR - i);
    }
}

static void GDB_STUB_SECTION_TEXT test_minmac() {
    for (uint8_t i = 0; i < 4; ++i) {
        miniMAC_rx_static_slot_allocate();
    }

    miniMAC_control(true, true);

    for (uint8_t i = 0; i < 5; ++i) {
        uint8_t* ptx_slot = miniMAC_tx_slot_allocate((12 * sizeof(uint32_t) - 2) * (i + 1));
        miniMAC_slot_complite_and_send(ptx_slot);
        while (!(IRQ_STATUS & (1 << IRQ_MINIMAC_TX)));
        IRQ_STATUS = (1 << IRQ_MINIMAC_TX);
        while (!(IRQ_STATUS & (1 << IRQ_MINIMAC_RX)));
        IRQ_STATUS = (1 << IRQ_MINIMAC_RX);
    }
}

void GDB_STUB_SECTION_TEXT start_tests() {
#ifdef SIM_TEST_FREQMETER
    start_freqmeter();
#endif

#ifdef SIM_TEST_MULTIPLICATION
    test_multiplication();
#endif

#ifdef SIM_TEST_MDIO
    test_mdio();
#endif

#ifdef SIM_TEST_MINIMAC
    test_minmac();
#endif
}
