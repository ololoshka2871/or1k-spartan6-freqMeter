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

#include "ds1338z.h"
#include "GPIO.h"
#include "crc32.h"

#include "../ETH_config.h"

#include "settings.h"

struct sSettings settings;

static void reset_MAC() {
    //----- SET OUR ETHENET UNIQUE MAC ADDRESS -----
    settings.MAC_ADDR[0] = ETH_MAC0;
    settings.MAC_ADDR[1] = ETH_MAC1;
    settings.MAC_ADDR[2] = ETH_MAC2;
    settings.MAC_ADDR[3] = ETH_MAC3;
    settings.MAC_ADDR[4] = ETH_MAC4;
    settings.MAC_ADDR[5] = ETH_MAC5;
}

static void reset_ip_settings() {

#ifdef DHCP_ON_STARTUP
    // dhcp will try to get ip addr
    settings.DHCP = 1;
#else
    settings.DHCP = 0;
#endif

    settings.IP_addr.u8[0] = ETH_IP0; //MSB
    settings.IP_addr.u8[1] = ETH_IP1;
    settings.IP_addr.u8[2] = ETH_IP2;
    settings.IP_addr.u8[3] = ETH_IP3; //LSB
    settings.IP_mask.u8[0] = ETH_NETMASK0; //MSB
    settings.IP_mask.u8[1] = ETH_NETMASK1;
    settings.IP_mask.u8[2] = ETH_NETMASK2;
    settings.IP_mask.u8[3] = ETH_NETMASK3; //LSB
    settings.IP_gateway.u8[0] = ETH_GW0;
    settings.IP_gateway.u8[1] = ETH_GW1;
    settings.IP_gateway.u8[2] = ETH_GW2;
    settings.IP_gateway.u8[3] = ETH_GW3;

    reset_MAC();
}

void Settings_init() {
    ds1338z_init();
    Settings_read();
}

void Settings_validate() {
    // validate net settings
    if ((!settings.IP_addr.u8[0]) || (!settings.IP_addr.u8[3]))
        goto __ip_reset;
    if ((settings.IP_addr.u8[0] == 0xff) || (settings.IP_addr.u8[3] == 0xff))
        goto __ip_reset;

    uint8_t zeros = 1;
    for(uint8_t i = 0; i < 32; ++i) {
        if (settings.IP_mask.u32 & (1 << i)) {
            zeros = 0;
        } else {
            if (!zeros)
                goto __ip_reset;
        }
    }

    if (settings.MAC_ADDR[0] & 0b11)
        goto __ip_reset;

    // all ok
    return;

__ip_reset:
    reset_ip_settings();

    // recalc crc32
    settings.CRC32 = 0;
    settings.CRC32 = crc32(&settings, sizeof(settings), 0);
}

void Settings_read() {
    ds1338z_readNVRAM(&settings, DS_1338Z_NVRAM_BASE, sizeof(settings));

    uint32_t crc = settings.CRC32;
    settings.CRC32 = 0;
    if (crc32(&settings, sizeof(settings), 0) != crc) {
        // reset to default
        reset_ip_settings();
        settings.CRC32 = crc32(&settings, sizeof(settings), 0);
        Settings_write();
    } else {
#ifdef MAC_ADDR_FORCE
        reset_MAC();
        settings.CRC32 = crc32(&settings, sizeof(settings), 0);
#else
        settings.CRC32 = crc;
#endif
        // crc ok, settings - ok
    }
}

void Settings_write() {
    Settings_validate();
    ds1338z_writeNVRAM(DS_1338Z_NVRAM_BASE, &settings, sizeof(settings));
}
