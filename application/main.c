/****************************************************************************
 * app_main.c
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

#include <string.h>

#include "irq.h"
#include "freqmeters.h"
#include "serial.h"
#include "minimac.h"
#include "mdio.h"
#include "microip.h"

static uint32_t irqCountes[FREQMETERS_COUNT];

void DELAY() {
    for (int i = 0; i < 100000; ++i)
        asm volatile("l.nop");
}

static const uint16_t measure_time_ms = 10;

void Send_Data() {
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        irq_disable(IRQ_FREQMETERS);
        uint32_t irqs = fm_getIRQCount(i);
        if (irqs != irqCountes[i]) {
            irq_enable(IRQ_FREQMETERS);
            irqCountes[i] = irqs;
            serial1_putchar('#');
            serial1_putchar(i);
            for (uint8_t j = 0; j < sizeof(uint32_t); ++j) {
                serial1_putchar(((uint8_t*)&irqs)[j]);
            }
            serial1_putchar('$');
        } else {
            irq_enable(IRQ_FREQMETERS);
        }
    }
}

void main(void)
{
    interrupts_init();

    fm_init();

    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        fm_setChanelReloadValue(i, 100, false);
        fm_enableChanel(i, true);
    }

    // find phy addr
    int8_t phy_addr = MDIO_DetectPHY(0);
    volatile uint16_t data[PHY_ANNPTR + 1];

    memset((void*)data, 0, sizeof(data));
    //MDIO_WriteREG(phy_addr, PHY_BMCR, PHY_BMCR_SPEED100MB | PHY_BMCR_FULL_DUPLEX);
    if (phy_addr >= 0) {
        for (uint8_t i = PHY_BMCR; i <= PHY_ANNPTR; ++i) {
            data[i] = MDIO_ReadREG_sync(phy_addr, i);
        }
    }

    microip_start(IPTOINT(192, 168, 1, 99));

    irq_enable(IS_FREQMETERS);

    EXIT_CRITICAL();

    while(1) {
        //Send_Data();
        microip_service();
    }
}
