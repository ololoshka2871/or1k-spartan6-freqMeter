/****************************************************************************
 * freqmeters.c
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

#include <string.h>

#include "irq.h"

#include "freqmeters.h"

static struct freqmeter_chanel freqmeters[FREQMETERS_COUNT];

static void fm_isr_handler(unsigned int *registers) {
    (void)registers;

    uint32_t chanels_to_scan = FM_IE & FM_IF;
    for (uint8_t ch = 0; ch < FREQMETERS_COUNT; ++ch) {
        if (chanels_to_scan & (1 << ch)) {

        }
    }
}

static void start_cycle(uint8_t chanel_num) {
    struct freqmeter_chanel* chanel = &freqmeters[chanel_num];
    chanel->newReload_vals[2] = chanel->newReload_vals[1];
    uint32_t cycleval = chanel->newReload_vals[0];
    if (!cycleval)
        cycleval = 1;
    chanel->newReload_vals[1] = cycleval;
    FM_RELOAD_CH(chanel_num, cycleval);
    FM_IE |= 1 << chanel_num;
}

void fm_init() {
    const struct freqmeter_chanel init_value = {
        .newReload_vals = {1, 1, 1},
        .enabled = 0,
    };
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        memcpy(&freqmeters[i], &init_value, sizeof(struct freqmeter_chanel));
    }

    set_irq_handler(IS_FREQMETERS, fm_isr_handler);
}



void update_chanel(uint8_t chanel) {
    const uint32_t chanel_mask = 1 << chanel;
    if ((FM_IE & chanel_mask) || freqmeters[chanel].enabled == 0) {
        // disable
        FM_IE &= ~chanel_mask;
    }
    if (freqmeters[chanel].enabled) { // update/restart
        start_cycle(chanel);
    }
}


