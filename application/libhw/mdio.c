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

#include "mdio.h"

/// MDIO operations at front of mdclk
/// http://www.ti.com/lit/ds/symlink/dp83848-ep.pdf
/// MSB first


enum enMDIOOperations {
    MDIO_READ = 0b01,
    MDIO_WRITE = 0b10
};

static int32_t last_detected_phy_addr = -1;

uint16_t MDIO_ReadREG_sync(int8_t phy_addr, const uint8_t reg_addr) {
    if (phy_addr < 0) {
        if (last_detected_phy_addr < 0) {
            if (MDIO_DetectPHY(0) < 0)
                return 0;
        }
        phy_addr = last_detected_phy_addr;
    }

    while(!(MDIO_REG_CTL & MDIO_REG_CTL_IF)); // wait ready
    MDIO_REG_CTL = MDIO_REG_CTL_START |
        ((phy_addr << MDIO_REG_CTL_PHY_ADDR_SH) & MDIO_REG_CTL_PHY_ADDR_MSK) |
        ((reg_addr << MDIO_REG_CTL_PHY_REGADDR_SH) & MDIO_REG_CTL_PHY_REGADDR_MSK);
    while(!(MDIO_REG_CTL & MDIO_REG_CTL_IF)); // wait ready
    return MDIO_REG_DATA;
}

void MDIO_WriteREG(int8_t phy_addr, const uint8_t reg_addr, const uint16_t val) {
    if (phy_addr < 0) {
        if (last_detected_phy_addr < 0) {
            if (MDIO_DetectPHY(0) < 0)
                return;
        }
        phy_addr = last_detected_phy_addr;
    }

    while(!(MDIO_REG_CTL & MDIO_REG_CTL_IF)); // wait ready
    MDIO_REG_DATA = val;
    MDIO_REG_CTL = MDIO_REG_CTL_START | MDIO_REG_CTL_RW |
        ((phy_addr << MDIO_REG_CTL_PHY_ADDR_SH) & MDIO_REG_CTL_PHY_ADDR_MSK) |
        ((reg_addr << MDIO_REG_CTL_PHY_REGADDR_SH) & MDIO_REG_CTL_PHY_REGADDR_MSK);
}

int8_t MDIO_DetectPHY(uint8_t startAddr) {
    int8_t phy_addr = -1;
    for (uint8_t i = startAddr; i < (1 << 6); ++i) {
        uint16_t v = MDIO_ReadREG_sync(i, PHY_BMCR);
        if(v != 0xffff) {
            phy_addr = i;
            last_detected_phy_addr = i;
            break;
        }
    }
    return phy_addr;
}

uint32_t MDIO_getConnectionStatus(int8_t phy_addr) {
    return ((MDIO_ReadREG_sync(phy_addr, PHY_BMSR) & PHY_BMSR_LinkOk) ? 1 : 0);
}
