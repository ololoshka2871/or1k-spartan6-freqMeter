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

#include "minmac.h"
#include "ethernet_phy.h"

/// MDIO operations at front of mdclk
/// http://www.ti.com/lit/ds/symlink/dp83848-ep.pdf
/// MSB first

#define PHY_ADDR_LEN_BITS        (5)
#define PHY_REG_ADDR_LEN_BITS    (5)

#define MDIO_DELAY_0()             mdio_delay()
#define MDIO_DELAY_1()             // mdio_delay()

enum enMDIOOperations {
    MDIO_READ = 0b01,
    MDIO_WRITE = 0b10
};

static void mdio_delay() {
    asm volatile("l.nop");
}

static void MDIO_send_bit(bool v) {
    MINIMAC_MDIO_BB = (v ? MINIMAC_MDIO_BB_DO : 0) | MINIMAC_MDIO_BB_OE;
    MDIO_DELAY_0();
    MINIMAC_MDIO_BB = (v ? MINIMAC_MDIO_BB_DO : 0) | MINIMAC_MDIO_BB_OE
            | MINIMAC_MDIO_BB_CLK;
    MDIO_DELAY_1();
}


static bool MDIO_read_bit() {
    MINIMAC_MDIO_BB = 0;
    MDIO_DELAY_0();
    MINIMAC_MDIO_BB = MINIMAC_MDIO_BB_CLK;
    MDIO_DELAY_1();
    return !!(MINIMAC_MDIO_BB & MINIMAC_MDIO_BB_DI);
}


static bool MDIO_write_z() {
    MINIMAC_MDIO_BB = 0;
    MDIO_DELAY_0();
    MINIMAC_MDIO_BB = MINIMAC_MDIO_BB_CLK;
    MDIO_DELAY_1();
}


static void MDIO_send_bits(const uint16_t v, uint8_t size) {
    while (size--)
        MDIO_send_bit(v & (1 << size));
}


static void
MDIO_prepare(const uint8_t phy_addr,
             const uint8_t reg_addr,
             const enum enMDIOOperations op) {
    MDIO_write_z();
    MDIO_send_bit(0);
    MDIO_send_bit(1);
    MDIO_send_bit(op & (1 << 0)); // read/write
    MDIO_send_bit(op & (1 << 1)); //
    MDIO_send_bits(phy_addr, PHY_ADDR_LEN_BITS); // phy address
    MDIO_send_bits(reg_addr, PHY_REG_ADDR_LEN_BITS); // phy register
}


uint16_t miniMAC_MDIO_ReadREG(const uint8_t phy_addr,
                              const uint8_t reg_addr) {
    MDIO_prepare(phy_addr, reg_addr, MDIO_READ);

    MDIO_write_z();  //pulse z
    MDIO_read_bit(); // dumy read

    uint16_t result = 0;
    uint8_t i = 8 * sizeof(uint16_t);
    while(i--) {
        if (MDIO_read_bit())
            result |= 1 << i;
    }

    MDIO_write_z();  //pulse z
    MINIMAC_MDIO_BB = 0;
}


void miniMAC_MDIO_WriteREG(const uint8_t phy_addr,
                           const uint8_t reg_addr,
                           const uint16_t val) {
    MDIO_prepare(phy_addr, reg_addr, MDIO_WRITE);

    MDIO_send_bit(1);    // send 1
    MDIO_send_bit(1);    // send 0

    MDIO_send_bits(val, 8 * sizeof(uint16_t));

    MDIO_write_z();  //pulse z
    MINIMAC_MDIO_BB = 0;
}

void miniMAC_MDIO_init() {
    // send 32 ones
    for (uint8_t i = 0; i < 32; ++i)
        MDIO_send_bit(1);
    MDIO_write_z();  //pulse z
    MINIMAC_MDIO_BB = 0;
}
