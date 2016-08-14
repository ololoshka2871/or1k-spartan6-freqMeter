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
static uint32_t alive_flags;

static void reload_cycle(uint8_t chanel_num) {
    struct freqmeter_chanel* chanel = &freqmeters[chanel_num];
    chanel->newReload_vals[2] = chanel->newReload_vals[1];
    uint32_t cycleval = chanel->newReload_vals[0];
    if (!cycleval)
        cycleval = 1;
    chanel->newReload_vals[1] = cycleval;
    FM_RELOAD_CH(chanel_num, cycleval);
}

static void fm_isr_handler(unsigned int *registers) {
    (void)registers;

    uint32_t chanels_to_scan = FM_IE & FM_IF;
    alive_flags |= chanels_to_scan;
    for (uint8_t ch = 0; ch < FREQMETERS_COUNT; ++ch) {
        if (chanels_to_scan & (1 << ch)) {
            freqmeters[ch].res_start_v = FM_START_VAL_CH(ch);
            freqmeters[ch].res_stop_v = FM_STOP_VAL_CH(ch);
            reload_cycle(ch);
        }
    }
}

void fm_init() {
    struct freqmeter_chanel init_value;
    init_value.enabled = 0;
    init_value.newReload_vals[0] = 1;
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        memcpy(&freqmeters[i], &init_value, sizeof(struct freqmeter_chanel));
    }

    alive_flags = 0;
    set_irq_handler(IS_FREQMETERS, fm_isr_handler);
}

void fm_updateChanel(uint8_t chanel) {
    const uint32_t chanel_mask = 1 << chanel;
    if ((FM_IE & chanel_mask) || freqmeters[chanel].enabled == 0) {
        // disable
        FM_IE &= ~chanel_mask;
    }
    if (freqmeters[chanel].enabled) { // update/restart
        reload_cycle(chanel);
        FM_IE |= 1 << chanel;
    }
}

void fm_enableChanel(uint8_t chanel, bool enable) {
    freqmeters[chanel].enabled = enable;
    fm_updateChanel(chanel);
}

void fm_setChanelReloadValue(uint8_t chanel, uint32_t reload_value,
                             bool force_restart) {
    freqmeters[chanel].newReload_vals[0] = reload_value;
    if (force_restart)
        fm_updateChanel(chanel);
}

uint32_t fm_getActualMeasureTime(uint8_t chanel) {
    uint32_t r = freqmeters[chanel].res_stop_v -
                 freqmeters[chanel].res_start_v;
    return r & (1 << 31) ? ~r + 1 : r;
}

uint32_t fm_GetMeasureTimestamp(uint8_t chanel) {
    return freqmeters[chanel].res_stop_v;
}

bool fm_checkAlive(uint8_t chanel) {
    uint32_t af = alive_flags;
    alive_flags &= ~(1 << chanel);
    return af & (1 << chanel);
}

uint32_t fm_getActualReloadValue(uint8_t chanel) {
    return freqmeters[chanel].newReload_vals[2];
}