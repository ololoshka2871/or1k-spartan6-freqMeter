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

#include <stdint.h>

#include "i2c.h"

#include "ds1338z.h"

#define DS1337Z_I2C_ADDR        (0b1101000)

static uint16_t bcd2bin(uint8_t c, uint8_t cm10) {
    return c + cm10 * 10;
}

static void bin2bcd(uint16_t v, uint8_t* c, uint8_t* cm10) {
    *cm10 = v / 10;
    *c = v % 10;
}

enum enDS1338z_err ds1338z_init() {
    i2c_init();

    uint8_t t;
    return (i2c_read_bus(DS1337Z_I2C_ADDR, 0x00, &t) == rsp_done) ?
                DS1338Z_OK : DS1338Z_ERROR;
}

enum enDS1338z_err ds1338z_getRawClockData(struct sDS1338z_clock_data *p) {
    return i2c_read_bus_mul(DS1337Z_I2C_ADDR, 0, (uint8_t*)p,
                            sizeof(struct sDS1338z_clock_data)) ?
                DS1338Z_OK : DS1338Z_ERROR;
}

enum enDS1338z_err ds1338z_getGetUnixTime(struct tm *tm) {
    struct sDS1338z_clock_data rawData;

    if (ds1338z_getRawClockData(&rawData) != DS1338Z_OK)
        return DS1338Z_ERROR;
    tm->tm_sec = bcd2bin(rawData.Seconds, rawData.Sec10);
    tm->tm_min = bcd2bin(rawData.Min, rawData.Min10);
    tm->tm_hour = bcd2bin(rawData.Hours, rawData.Hours10) + rawData.AM_PM * 20;
    tm->tm_mday = bcd2bin(rawData.Day, rawData.Day10);
    tm->tm_wday = rawData.day_of_week;
    tm->tm_year = bcd2bin(rawData.Year, rawData.Year10) + 2000;

    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_setRawClockData(const struct sDS1338z_clock_data *p) {
    return i2c_write_bus_mul(DS1337Z_I2C_ADDR, 0, (uint8_t*)p,
                             sizeof(struct sDS1338z_clock_data)) ?
                DS1338Z_OK : DS1338Z_ERROR;
}

enum enDS1338z_err ds1338z_setGetUnixTime(const struct tm *tm) {
    struct sDS1338z_clock_data rawData;

    uint8_t c, cm10;

    bin2bcd(tm->tm_sec, &c, &cm10);
    rawData.Seconds = c;
    rawData.Sec10 = cm10;

    bin2bcd(tm->tm_min, &c, &cm10);
    rawData.Min = c;
    rawData.Min10 = cm10;

    bin2bcd(tm->tm_hour, &c, &cm10);
    rawData.Hours = c;
    rawData.Hours10 = cm10 / 10;
    rawData.AM_PM = cm10 / 20;

    rawData.day_of_week = tm->tm_wday;

    bin2bcd(tm->tm_mday, &c, &cm10);
    rawData.Day = c;
    rawData.Day10 = cm10;

    bin2bcd(tm->tm_year - 2000, &c, &cm10);
    rawData.Year = c;
    rawData.Year10 = cm10;

    return ds1338z_setRawClockData(&rawData);
}
