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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include "irq.h"
#include "freqmeters.h"
#include "GPIO.h"
#include "seg7_display.h"
#include "serial.h"

static uint32_t timestamps[FREQMETERS_COUNT];

void DELAY() {
    for (int i = 0; i < 100000; ++i)
        asm volatile("l.nop");
}

void Send_Data() {
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        uint32_t ts = fm_GetMeasureTimestamp(i);
        if (ts != timestamps[i]) {
            timestamps[i] = ts;
            uint32_t periods = fm_getActualReloadValue(i);
            uint32_t value   = fm_getActualMeasureTime(i);
            if ((!value) || (!periods))
                continue;
            double F = (double)value * (double)periods;

            serial1_putchar('#');
            serial1_putchar(i);
            serial1_putchar('=');
            for (uint8_t j = 0; j < sizeof(double); ++j) {
                serial1_putchar(((uint8_t*)&F)[j]);
            }
            serial1_putchar('$');
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

    GPIO portA = gpio_port_init(GPIO_PORTA, 0b1111);
    uint8_t v = 1;
    uint16_t count = 0;

    irq_enable(IS_FREQMETERS);

    EXIT_CRITICAL();

    seg7_PutStr("1234", 4, ' ');
    while(1) {
        //DELAY();
        if (v == 1 << 4) v = 1;
        gpio_port_set_all(portA, ~v);
        seg7_printHex(fm_getActualMeasureTime(count));
        for (uint8_t i = 0; i < 4; ++i) {
            seg7_dpSet(seg7_num2Segment(i), v & (1 << i));
        }
        v <<= 1;
        count = (count + 1) % FREQMETERS_COUNT;
        Send_Data();
        //fm_updateChanel(count);
    }
}
