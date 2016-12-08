/****************************************************************************
 *
 *   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
 *   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 *   Based on iicmb: http://opencores.org/project,iicmb
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

#include <sys/time.h>
#include <string.h>
#include <errno.h>
#include <stddef.h>
#include <irq.h>

#include "prog_timer.h"

#include "ds1338z.h"

#include "rtc.h"

#ifndef BULD_TIMESTAMP
#error "Macro \"BULD_TIMESTAMP\" mast me defined!"
#endif

#define MS_IN_S                 1000
#define NS_IN_MS                1000000
#define MS_2_NS(ms)             ((ms) * NS_IN_MS)
#define NS_2_MS(ns)             ((ns) / NS_IN_MS)

static time_t local_seconds;

static void clock_add_second(void* cookie) {
    (void)cookie;
    ++local_seconds;
}

void rtc_init() {
    progtimer_new(PROGTIMER_TICKS_IN_SEC, clock_add_second, NULL);

    // sync system clock with external
    struct timespec current_time;
    if (ds1338z_init() != DS1338Z_OK)
        return;
    if (ds1338z_getUnixTime(&current_time.tv_sec) != DS1338Z_OK)
        return;
    if (current_time.tv_sec < BULD_TIMESTAMP)
        current_time.tv_sec = BULD_TIMESTAMP;
    current_time.tv_nsec = 0;
    clock_settime(0, &current_time);
}

int clock_gettime(clockid_t clockid, struct timespec *ts) {
    (void)clockid;
    clock_catch_inpure_timestamp(ts);
    clock_purify_time(ts);
    return 0;
}

void clock_catch_inpure_timestamp(struct timespec *ts) {
    ts->tv_sec = local_seconds;
    ts->tv_nsec = progtimer_get_ticks();
}

void clock_purify_time(struct timespec *ts) {
    ts->tv_nsec = MS_2_NS(ts->tv_nsec % MS_IN_S);
}

int clock_settime(clockid_t clockid, const struct timespec *ts) {
    (void)clockid;

    if (ts->tv_sec < BULD_TIMESTAMP)
        return -EINVAL;

    progtimer_setclock(NS_2_MS(ts->tv_nsec) % PROGTIMER_TICKS_IN_SEC);
    local_seconds = ts->tv_sec;
    return ds1338z_setUnixTime(&ts->tv_sec) == DS1338Z_OK ? 0 : -ECONNREFUSED;
}
