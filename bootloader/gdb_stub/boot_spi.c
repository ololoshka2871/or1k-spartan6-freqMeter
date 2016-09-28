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

#include "boot_spi.h"

#define  JEDEC_ID_COMMAND        0x9f
#define  DATA_READ_COMMAND       0x03
#define  DATA_READ_FAST_COMMAND  0x0B

static void GDB_STUB_SECTION_TEXT wait_transmitted() {
    while (!(SPI_STATUS & TXE));
}

void GDB_STUB_SECTION_TEXT
boot_spi_init(enum enSPIClockDevider clockDevider, enum enSPIMode mode) {
    SPI_STATUS = 0; // no interrupts
    SPI_CTRL = mode;
    SPI_BD = clockDevider;

    wait_transmitted();
}

void GDB_STUB_SECTION_TEXT boot_spi_disable() {
}

enum enSpiErr GDB_STUB_SECTION_TEXT boot_spi_read(uint8_t* dest) {
    if (!(SPI_STATUS & TXE))
        return SPI_NO_DATA_AVALABLE;
    else {
        *dest = SPI_SR;
        return SPI_OK;
    }
}

enum enSpiErr GDB_STUB_SECTION_TEXT boot_spi_write(uint8_t byte_to_transmitt) {
    if (!(SPI_STATUS & TXR))
        return SPI_FIFO_FULL;
    else {
        SPI_DR = byte_to_transmitt;
        return SPI_OK;
    }
}

uint8_t GDB_STUB_SECTION_TEXT
boot_spi_transfer_byte(uint8_t byte_to_transmitt) {
    wait_transmitted();

    SPI_DR = byte_to_transmitt;
    while (!(SPI_STATUS & TXE)); // wait for transfer
    return SPI_SR;
}

void GDB_STUB_SECTION_TEXT
boot_spi_transfer_buf(const uint8_t* txp, uint8_t *rxp, uint8_t len) {
    wait_transmitted();

    SPI_DR = *txp++;
    if (len > 1) {
        SPI_DR = *txp++;
        for (uint8_t i = 2; i < len; i++) {
            uint8_t rx, tx = *txp++;
            while (!(SPI_STATUS & TXR));
            rx = SPI_DR;
            SPI_DR = tx;
            *rxp++ = rx;
        }
        while (!(SPI_STATUS & TXR));
        *rxp++ = SPI_DR;
    }
    while (!(SPI_STATUS & TXE));
    *rxp++ = SPI_SR;
}

uint8_t GDB_STUB_SECTION_TEXT spi_probe_flash(uint8_t cs_from, uint8_t cs_to) {
    static const uint8_t cmd[] = {JEDEC_ID_COMMAND, 1, 2, 3, 4};
    struct Flash_ID result;

    boot_spi_init(SPI_CLOCK_DEV_256, SPI_MODE3);
    while(cs_from <= cs_to) {
        SPI_CS_SEL = cs_from;

        boot_spi_transfer_buf(cmd, (uint8_t*)&result, sizeof(cmd));

        SPI_CS_SEL = 0;

        if (result.Manufacturer && (result.Manufacturer != 0xff)) {
            result.cs_found = cs_from;
            break;
        } else { // not found
            result.cs_found = 0;
        }

        ++cs_from;
    }
    boot_spi_disable();

    return result.cs_found;
}

void GDB_STUB_SECTION_TEXT
spi_flash_read(uint8_t cs_num, uint32_t offset, uint8_t* dest, uint32_t size) {
    boot_spi_init(SPI_CLOCK_DEV_32, SPI_MODE3);
    SPI_CS_SEL = cs_num;

    uint8_t command[] = {
        size > 2 ? DATA_READ_FAST_COMMAND : DATA_READ_COMMAND,
        offset >> 16,
        offset >> 8,
        offset,
        1, 2, 3
    };
    if (size <= 2) {
        boot_spi_transfer_buf(command, command, sizeof(command - 1));
        *dest++ = command[4];
        if (size == 2)
            *dest = command[5];
    } else {
        int i;
        wait_transmitted();

        for (i = 0; i < sizeof(command); ++i) {
            while (!(SPI_STATUS & TXR));
            SPI_DR = command[i];
        }
        for (i = 0; i < size - 2; i++) {
            uint8_t rx;
            while (!(SPI_STATUS & TXR));
            rx = SPI_DR;
            SPI_DR = 0xA5;
            *dest++ = rx;
        }
        while (!(SPI_STATUS & TXR));
        *dest++ = SPI_DR;
        while (!(SPI_STATUS & TXE));
        *dest = SPI_SR;
    }

    SPI_CS_SEL = 0;
}
