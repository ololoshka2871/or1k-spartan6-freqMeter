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

#include "bcd.h"

#include "i2c.h"

#include "ds1338z.h"

#define DS1338Z_I2C_ADDR        (0b1101000)
#define DS1338Z_NVRAM_START     (0x08)

#define SECONDS_IN_MINUTE       (60)
#define MINUTES_IN_HOUR         (60)
#define HOURS_IN_DAY            (24)
#define MONTHS_IN_YEAR          (sizeof(daysinmonth))
#define DAYS_IN_YEAR            (365)
#define UNIX_MILENUUM           ((time_t)946684800UL)

#ifdef I2C_DISABLED

enum enDS1338z_err ds1338z_init() { return DS1338Z_OK; }
enum enDS1338z_err ds1338z_getRawClockData(struct sDS1338z_clock_data *p) {
    p->day = 1;
    p->DoW = 1;
    p->hour = 0;
    p->minute = 0;
    p->month = 1;
    p->second = 0;
    p->year = 0;
    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_getUnixTime(time_t *tm) {
    *tm = UNIX_MILENUUM;
    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_setRawClockData(const struct sDS1338z_clock_data *p) {
    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_setUnixTime(const time_t *tm) {
    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_readNVRAM(void *dest, uint8_t offset, uint8_t size) {
    memset(dest, 0, size);
    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_writeNVRAM(uint8_t offset, void *src, uint8_t size) {
    return DS1338Z_OK;
}

#else

const uint8_t daysinmonth[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };


enum enDS1338z_err ds1338z_init() {
    i2c_init();

    uint8_t t;
    return (i2c_read_bus_mul(DS1338Z_I2C_ADDR, 0x00, &t, 1) == rsp_done) ?
                DS1338Z_OK : DS1338Z_ERROR;
}

enum enDS1338z_err ds1338z_getRawClockData(struct sDS1338z_clock_data *p) {
    if (i2c_read_bus_mul(DS1338Z_I2C_ADDR, 0, (uint8_t*)p,
                            sizeof(struct sDS1338z_clock_data)) != rsp_done)
        return DS1338Z_ERROR;

    for (uint8_t i = 0; i < sizeof(struct sDS1338z_clock_data); ++i)
        ((uint8_t*)p)[i] = u8_bcd2dec(((uint8_t*)p)[i]);
    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_getUnixTime(time_t *tm) {
    struct sDS1338z_clock_data rawData;

    if (ds1338z_getRawClockData(&rawData) != DS1338Z_OK)
        return DS1338Z_ERROR;

    timer_t days = 0;
    for(uint8_t year = 0; year < rawData.year; ++year) { // year - 2000
        days += (year % 4  == 0) ? (DAYS_IN_YEAR + 1) : DAYS_IN_YEAR;
    }
    for(uint8_t month = 0; month < rawData.month - 1; ++month) {
        days += daysinmonth[month];
        if ((rawData.year % 4 == 0) && (month == 1))
            ++days;
    }

    time_t seconds_since_2k = (days + (time_t)(rawData.day - 1)) *
            HOURS_IN_DAY *  MINUTES_IN_HOUR * SECONDS_IN_MINUTE;
    seconds_since_2k += (time_t)rawData.second;
    seconds_since_2k += (time_t)rawData.minute * SECONDS_IN_MINUTE;
    seconds_since_2k += (time_t)rawData.hour * MINUTES_IN_HOUR * SECONDS_IN_MINUTE;

    *tm = seconds_since_2k + UNIX_MILENUUM;

    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_setRawClockData(const struct sDS1338z_clock_data *p) {
    for (uint8_t i = 0; i < sizeof(struct sDS1338z_clock_data); ++i)
        ((uint8_t*)p)[i] = u8_dec2bcd(((uint8_t*)p)[i]);

    if (i2c_write_bus_mul(DS1338Z_I2C_ADDR, 0, (uint8_t*)p,
                             sizeof(struct sDS1338z_clock_data)) != rsp_done)
        return DS1338Z_ERROR;

    return DS1338Z_OK;
}

enum enDS1338z_err ds1338z_setUnixTime(const time_t *tm) {
    struct sDS1338z_clock_data rawData;

    time_t seconds = *tm - UNIX_MILENUUM;

    for(uint8_t year = 0; year < 99; ++year) { // year - 2000
        time_t seconds_this_year = (year % 4  == 0) ?
                     ((DAYS_IN_YEAR + 1) * (HOURS_IN_DAY * MINUTES_IN_HOUR * SECONDS_IN_MINUTE))
                   : (DAYS_IN_YEAR * (HOURS_IN_DAY * MINUTES_IN_HOUR * SECONDS_IN_MINUTE));
        if (seconds_this_year <= seconds) {
            seconds -= seconds_this_year;
        } else {
            rawData.year = year;
            break;
        }
    }

    for (uint8_t month = 1; month <= MONTHS_IN_YEAR; ++month) {
        time_t seconds_this_month = daysinmonth[month - 1] * (HOURS_IN_DAY * MINUTES_IN_HOUR * SECONDS_IN_MINUTE);
        if ((month == 2) && (rawData.year % 4 == 0))
            seconds_this_month += (HOURS_IN_DAY * MINUTES_IN_HOUR * SECONDS_IN_MINUTE);
        if (seconds_this_month < seconds) {
            seconds -= seconds_this_month;
        } else {
            rawData.month = month;
            break;
        }
    }

    rawData.day = seconds / (HOURS_IN_DAY * MINUTES_IN_HOUR * SECONDS_IN_MINUTE);
    seconds -= rawData.day * (HOURS_IN_DAY * MINUTES_IN_HOUR * SECONDS_IN_MINUTE);
    ++rawData.day; // days 1 - 31

    rawData.hour = seconds / (MINUTES_IN_HOUR * SECONDS_IN_MINUTE);
    seconds -= rawData.hour * (MINUTES_IN_HOUR * SECONDS_IN_MINUTE);

    rawData.minute = seconds / SECONDS_IN_MINUTE;
    rawData.second = seconds - rawData.minute * SECONDS_IN_MINUTE;

    return ds1338z_setRawClockData(&rawData);
}

enum enDS1338z_err ds1338z_readNVRAM(void *dest, uint8_t offset, uint8_t size) {
    return (i2c_read_bus_mul(DS1338Z_I2C_ADDR,
                             DS1338Z_NVRAM_START + offset,
                             dest, size) == rsp_done) ?
                DS1338Z_OK : DS1338Z_ERROR;
}

enum enDS1338z_err ds1338z_writeNVRAM(uint8_t offset, void *src, uint8_t size) {
    return (i2c_write_bus_mul(DS1338Z_I2C_ADDR,  DS1338Z_NVRAM_START + offset,
                             src, size)) == rsp_done ?
                DS1338Z_OK : DS1338Z_ERROR;
}
#endif
