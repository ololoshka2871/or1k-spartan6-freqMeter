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
#include <stdio.h>
#include <assert.h>

#include "irq.h"
#include "freqmeters.h"
#include "mdio.h"
#include "microip.h"
#include "udp.h"

void DELAY() {
    for (int i = 0; i < 100000; ++i)
        asm volatile("l.nop");
}

static const uint16_t measure_time_ms = 10;
#if 0
static uint32_t irqCountes[FREQMETERS_COUNT];

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
#endif

void Send_Data(uint32_t chanel, double F, uint32_t per) {
    struct p {
        uint32_t ch;
        uint32_t per;
        double F;
    } _p = {chanel, per, F};

    send_udp_packet(IPTOINT(192, 168, 1, 25), 4999, 4998, &_p, sizeof(_p));
}

static uint32_t timestamps[FREQMETERS_COUNT];
static uint32_t starts[FREQMETERS_COUNT];
static uint32_t measure_time[FREQMETERS_COUNT];
static double Fs[FREQMETERS_COUNT];

static void Process_freqmeters() {
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        uint32_t ts = fm_getMeasureTimestamp(i);
        if (ts != timestamps[i]) {
            timestamps[i] = ts;
            irq_disable(IS_FREQMETERS);
            uint32_t periods = fm_getActualReloadValue(i);
            uint32_t value   = fm_getActualMeasureTime(i);
            starts[i] = fm_getMeasureStart_pos(i);
            measure_time[i] = fm_getActualReloadValue(i);
            irq_enable(IS_FREQMETERS);
            if ((!value) || (!periods))
                continue;
            double F = (double)periods / (double)value * F_REF;
            Fs[i] = F;
#if 0
            //recalc new reload value
            uint32_t reload_val = (uint32_t)(F * measure_time_ms / 1000);
            if (!reload_val)
                reload_val = 1;
            fm_setChanelReloadValue(i, reload_val, false); // set new reload val
#endif
        }
    }
}

static void cb_udp_callback(uint32_t src_ip, uint16_t src_port,
                            uint16_t dst_port, void *data,
                            uint32_t length) {
    assert(data);
#if 0
    char buff[(sizeof(uint32_t) + sizeof(double)) * FREQMETERS_COUNT];
    memcpy(buff, timestamps, sizeof(timestamps));
    memcpy(buff + sizeof(timestamps), Fs, sizeof(Fs));
#else
    struct {
        uint32_t start;
        uint32_t stop;
        uint32_t mt;
        double res;
    } r[FREQMETERS_COUNT];
    for (uint32_t i = 0; i < FREQMETERS_COUNT; ++i) {
        r[i].start = starts[i];
        r[i].stop = timestamps[i];
        r[i].mt = measure_time[i];
        r[i].res = Fs[i];
    }
#endif
    send_udp_packet(src_ip, src_port, dst_port, r, sizeof(r));
}

static void configure_ethernet_PHY() {
    // find phy addr
    int8_t phy_addr = MDIO_DetectPHY(0);
    if (phy_addr > 0) {
        // force 100 Mb/s FD

        MDIO_WriteREG(phy_addr, PHY_BMCR, PHY_BMCR_SPEED100MB | PHY_BMCR_FULL_DUPLEX);

        // set elastic bufer max len
        uint8_t v;
        v = MDIO_ReadREG_sync(phy_addr, PHY_RBR);
        MDIO_WriteREG(phy_addr, PHY_RBR, (v & ~(PHY_RBR_ELAST_BUF_MSK))
                      | PHY_RBR_RMII_REV1_0
                      | (0b00 << PHY_RBR_ELAST_BUF_SH));
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

    configure_ethernet_PHY();

    microip_start(IPTOINT(192, 168, 1, 99));
    set_rx_callback(cb_udp_callback);

    irq_enable(IS_FREQMETERS);

    EXIT_CRITICAL();

    while(1) {
        //Send_Data();
        Process_freqmeters();
        microip_service();
    }
}
