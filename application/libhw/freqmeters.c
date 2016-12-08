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

#include "freqmeters.h"

#ifndef BOOTLOADER
#include "irq.h"
#include "graycode.h"
#endif

#ifndef FREQMETER_MEASURE_TIME_MAX
#define FREQMETER_MEASURE_TIME_MAX 1000
#warning "Macro FREQMETER_MEASURE_TIME_MAX not defined, assuming 1000"
#endif

#ifndef SYSTEM_MEASURE_TIME_DEFAULT
#define SYSTEM_MEASURE_TIME_DEFAULT 10
#warning "Macro SYSTEM_MEASURE_TIME_DEFAULT not defined, assumong 10"
#endif

#ifndef F_REF
#error "Macro F_REF not defined!"
#endif

#define STARTUP_FREQUNCY    1000.0

static struct freqmeter_chanel freqmeters[FREQMETERS_COUNT];
static uint16_t measure_time_ms[FREQMETERS_COUNT];
static uint32_t alive_flags;

static void reload_cycle(uint8_t chanel_num) {
    struct freqmeter_chanel* chanel = &freqmeters[chanel_num];
    chanel->reloadVals.readyReload_val = chanel->reloadVals.inWorkReload_val;
    uint32_t cycleval = chanel->reloadVals.newReload_val;
    if (!cycleval)
        cycleval = 1;
    chanel->reloadVals.inWorkReload_val = cycleval;
    FM_RELOAD_CH(chanel_num, cycleval);
}

static void fm_isr_handler(unsigned int *registers) {
    (void)registers;
    uint32_t chanels_to_scan = FM_IE & FM_IF;
#if VERBOSE_DEBUG
    if (!chanels_to_scan)
        asm volatile("l.trap 0");
#endif
    alive_flags |= chanels_to_scan;
    for (uint8_t ch = 0; ch < FREQMETERS_COUNT; ++ch) {
        if (chanels_to_scan & 1) {
            freqmeters[ch].res_start_v = FM_START_VAL_CH(ch);
            freqmeters[ch].res_stop_v = FM_STOP_VAL_CH(ch);
            ++freqmeters[ch].irq_count;
#ifndef SIM
            clock_catch_inpure_timestamp(&freqmeters[ch].timestamp);
            freqmeters[ch].signal_present = !!(FM_SP & (1 << ch));
#endif
            reload_cycle(ch);
        }

        chanels_to_scan >>= 1;
        if (!chanels_to_scan)
            break;      
    }
}


static uint32_t measure_time_ms2ticks(double F, uint16_t _measure_time_ms) {
#ifndef SIM
    if (_measure_time_ms > FREQMETER_MEASURE_TIME_MAX)
        _measure_time_ms = FREQMETER_MEASURE_TIME_MAX;
    uint32_t reload_val = (uint32_t)(F * _measure_time_ms / 1000);
    if (!reload_val)
        reload_val = 1;
    return reload_val;
#else
    return 0;
#endif
}

void fm_init() {
    struct freqmeter_chanel init_value;
    init_value.enabled = 0;
    init_value.reloadVals.newReload_val = 1;
    init_value.irq_count = 0;
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        memcpy(&freqmeters[i], &init_value, sizeof(struct freqmeter_chanel));
        freqmeters[i].enabled = true;
        freqmeters[i].reloadVals.newReload_val =
                measure_time_ms2ticks(STARTUP_FREQUNCY, SYSTEM_MEASURE_TIME_DEFAULT);
        measure_time_ms[i] = SYSTEM_MEASURE_TIME_DEFAULT;
    }

    alive_flags = 0;
#ifndef SIM
    set_irq_handler(IS_FREQMETERS, fm_isr_handler);
    irq_enable(IS_FREQMETERS);
#endif
}

void fm_updateChanel(uint8_t chanel) {
    const uint32_t chanel_mask = 1 << chanel;
    if ((FM_IE & chanel_mask) || freqmeters[chanel].enabled == 0) {
        // disable
        FM_IE &= ~chanel_mask;
    }
    if (freqmeters[chanel].enabled) { // update/restart
        reload_cycle(chanel);
        FM_IE |= chanel_mask;
    }
}

void fm_enableChanel(uint8_t chanel, bool enable) {
    freqmeters[chanel].enabled = enable;
    fm_updateChanel(chanel);
}

static void fm_setChanelReloadValue(uint8_t chanel, uint32_t reload_value,
                             bool force_restart) {
    freqmeters[chanel].reloadVals.newReload_val = reload_value;
    if (force_restart)
        fm_updateChanel(chanel);
}

uint32_t fm_getActualMeasureTime(uint8_t chanel) {
    uint32_t v = fm_getMeasureTimestamp(chanel) - fm_getMeasureStart_pos(chanel);
    if (v & (1 << 31))
        v = ((1ul << (SYSTEM_FREF_COUNTER_LEN)) - 1) -
            fm_getMeasureStart_pos(chanel) +
            fm_getMeasureTimestamp(chanel);
        return v;
}



static uint32_t hybrid2bin(uint32_t v) {
#if defined(BOOTLOADER) || !defined(MASTER_HYBRID_COUNTER)
    return v;
#else
    uint32_t binary = v & ~0b1111;
    uint32_t gray = v & 0b1111;
    return binary | gray2bin(gray);
#endif
}

uint32_t fm_getMeasureTimestamp(uint8_t chanel) {
    return hybrid2bin(freqmeters[chanel].res_stop_v);
}

uint32_t fm_getMeasureStart_pos(uint8_t chanel) {
    return hybrid2bin(freqmeters[chanel].res_start_v);
}

bool fm_checkAlive(uint8_t chanel) {
    uint32_t af = alive_flags;
    alive_flags &= ~(1 << chanel);
    return af & (1 << chanel);
}

uint32_t fm_getActualReloadValue(uint8_t chanel) {
    return freqmeters[chanel].reloadVals.readyReload_val;
}

uint32_t fm_getIRQCount(uint8_t chanel) {
    return freqmeters[chanel].irq_count;
}

void fm_process() {
    for (uint8_t chanel = 0; chanel < FREQMETERS_COUNT; ++chanel) {
        if (freqmeters[chanel].signal_present) {
#ifndef SIM
            irq_disable(IS_FREQMETERS);
#endif
            uint32_t periods = fm_getActualReloadValue(chanel);
            uint32_t value   = fm_getActualMeasureTime(chanel);
#ifndef SIM
            irq_enable(IS_FREQMETERS);
#endif

            if ((!value) || (!periods))
                continue;

            double F = (double)periods / (double)value * F_REF;
            freqmeters[chanel].F = F;

            freqmeters[chanel].reloadVals.newReload_val = measure_time_ms2ticks(F, measure_time_ms[chanel]);
        }
    }
}

bool fm_setMeasureTime(uint8_t chanel, uint16_t new_measure_time_ms) {
    if (new_measure_time_ms > FREQMETER_MEASURE_TIME_MAX)
        return false;
    measure_time_ms[chanel] = new_measure_time_ms;
    double F = (freqmeters[chanel].enabled && (freqmeters[chanel].F > 0)) ?
                freqmeters[chanel].F : SYSTEM_MEASURE_TIME_DEFAULT;
    fm_setChanelReloadValue(chanel, measure_time_ms2ticks(F, new_measure_time_ms), true);
    return true;
}
