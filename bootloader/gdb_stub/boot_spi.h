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

#ifndef BOOT_SPI_H
#define BOOT_SPI_H

#include <stdint.h>
#include "gdb-stub-sections.h"
#include "mem_map.h"

#define SPI_SR      (*(REG32 (SPI_BASE + 0x00)))
#define SPI_DR      (*(REG32 (SPI_BASE + 0x04)))
#define SPI_STATUS  (*(REG32 (SPI_BASE + 0x08)))
#define SPI_CTRL    (*(REG32 (SPI_BASE + 0x0C)))
#define SPI_BD      (*(REG32 (SPI_BASE + 0x10)))
#define SPI_CS_SEL  (*(REG32 (SPI_BASE + 0x14)))

enum enSPIClockDevider {
    SPI_CLOCK_DEV_2 = 0,
    SPI_CLOCK_DEV_4 = 1,
    SPI_CLOCK_DEV_16 = 2,
    SPI_CLOCK_DEV_32 = 3,
    SPI_CLOCK_DEV_8 = 4,
    SPI_CLOCK_DEV_64 = 5,
    SPI_CLOCK_DEV_128 = 6,
    SPI_CLOCK_DEV_256 = 7,
};

#define TXR          (1 << 1)
#define TXE          (1 << 0)
#define TXR_EN       (1 << 1)
#define TXE_EN       (1 << 0)
#define MODE_MASK    (0b11 << 0)
#define MODE_SHIFT   (0)

#define CPOL        (1 << (MODE_SHIFT + 1))      // Clock Polarity
#define CPHA        (1 << MODE_SHIFT)            // Clock Phase

enum enSpiErr {
    SPI_OK = 0,
    SPI_NO_DATA_AVALABLE = 1,
    SPI_FIFO_FULL = 2,
};

enum enSPIMode {
    SPI_MODE0 = 0,
    SPI_MODE1 = CPHA,
    SPI_MODE2 = CPOL,
    SPI_MODE3 = CPOL | CPHA,
};

struct Flash_ID {
    uint8_t dummy;
    uint8_t Manufacturer;
    uint8_t Memory_Type;
    uint8_t Capacity;

    uint8_t cs_found;
};

uint8_t spi_probe_flash(uint8_t cs_from, uint8_t cs_to);
void GDB_STUB_SECTION_TEXT spi_flash_read(uint8_t cs_num, uint32_t offset,
                    uint8_t* dest, uint32_t size);

#endif // BOOT_SPI_H
