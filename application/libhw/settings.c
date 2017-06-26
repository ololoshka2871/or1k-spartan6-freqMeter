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

#include <string.h>

#include "ds1338z.h"
#include "GPIO.h"
#include "crc32.h"

#include "ETH_config.h"

#include "settings.h"


#define REFERENCE_FREQ_OFFSET_MAX       (SYSTEM_FREQ_TYPE)50000.0
#define REFERENCE_FREQ_MIN              ((SYSTEM_FREQ_TYPE)DEVICE_REF_CLOCK_HZ - REFERENCE_FREQ_OFFSET_MAX)
#define REFERENCE_FREQ_MAX              ((SYSTEM_FREQ_TYPE)DEVICE_REF_CLOCK_HZ + REFERENCE_FREQ_OFFSET_MAX)


struct sSettings settings;



void settings_update_crc32(struct sSettings *settings) {
    settings->CRC32 = 0;
    settings->CRC32 = crc32(settings, sizeof(struct sSettings));
}


bool verify_settings_crc32(struct sSettings *settings) {
    uint32_t curentCRC32 = settings->CRC32;
    settings_update_crc32(settings);
    return curentCRC32 == settings->CRC32;
}


static void default_ip_settings(struct sSettings *settings) {
#ifdef DHCP_ON_STARTUP
    // dhcp will try to get ip addr
    settings->DHCP = 1;
#else
    settings->DHCP = 0;
#endif

    settings->IP_addr.u8[0] = ETH_IP0; //MSB
    settings->IP_addr.u8[1] = ETH_IP1;
    settings->IP_addr.u8[2] = ETH_IP2;
    settings->IP_addr.u8[3] = ETH_IP3; //LSB
    settings->IP_mask.u8[0] = ETH_NETMASK0; //MSB
    settings->IP_mask.u8[1] = ETH_NETMASK1;
    settings->IP_mask.u8[2] = ETH_NETMASK2;
    settings->IP_mask.u8[3] = ETH_NETMASK3; //LSB
    settings->IP_gateway.u8[0] = ETH_GW0;
    settings->IP_gateway.u8[1] = ETH_GW1;
    settings->IP_gateway.u8[2] = ETH_GW2;
    settings->IP_gateway.u8[3] = ETH_GW3;
}

static void default_MAC_settings(struct sSettings *settings) {
    //----- SET OUR ETHENET UNIQUE MAC ADDRESS -----
    settings->MAC_ADDR[0] = ETH_MAC0;
    settings->MAC_ADDR[1] = ETH_MAC1;
    settings->MAC_ADDR[2] = ETH_MAC2;
    settings->MAC_ADDR[3] = ETH_MAC3;
    settings->MAC_ADDR[4] = ETH_MAC4;
    settings->MAC_ADDR[5] = ETH_MAC5;
}

static void default_freqmeter_settings(struct sSettings *settings) {
    settings->ReferenceFrequency = (SYSTEM_FREQ_TYPE)DEVICE_REF_CLOCK_HZ;
}

void Settings_defaults(struct sSettings *settings) {
    default_ip_settings(settings);
    default_MAC_settings(settings);
    default_freqmeter_settings(settings);
    settings_update_crc32(settings);
}


void Settings_read(struct sSettings *settings) {
    ds1338z_readNVRAM(settings, DS_1338Z_NVRAM_BASE, sizeof(struct sSettings));
}


void Settings_write(struct sSettings *settings) {
    ds1338z_writeNVRAM(DS_1338Z_NVRAM_BASE, settings, sizeof(struct sSettings));
}


void Settings_init() {
    if (ds1338z_init() == DS1338Z_OK) {
        Settings_read(&settings);
    }
    Settings_validate(&settings, Settings_defaults);
#ifdef MAC_ADDR_FORCE
    default_MAC_settings(&settings);
    settings_update_crc32(&settings);
#endif
    Settings_write(&settings);
}

static bool isIPInvalid(union IP_ADDR ip) {
    return (!ip.u8[0]) || (!ip.u8[3]) || (ip.u8[0] == 0xff) || (ip.u8[3] == 0xff);
}

enum enSettingsValidatorError
Settings_validate(struct sSettings *validateing_object, validate_restorer restorer) {
    enum enSettingsValidatorError result = SV_ERR_OK;
    struct sSettings restored;
    restorer(&restored);
    if (!verify_settings_crc32(&restored)) {
        // restored settings incorrect, set restored to default
        Settings_defaults(&restored);
    }

    if (!verify_settings_crc32(validateing_object)) {
        // curent settings incorrect, restoring all
        memcpy(validateing_object, &restored, sizeof(struct sSettings));
        result |= SV_ERR_CRC;
    } else {
        if (isIPInvalid(validateing_object->IP_addr)) {
            // incorrect ip, restoring
            validateing_object->IP_addr = restored.IP_addr;
            result |= SV_ERR_IP;
        }

        {   // check netmask
            uint8_t zeros = 1;
            for(uint8_t i = 0; i < sizeof(union IP_ADDR) * 8; ++i) {
                if (validateing_object->IP_mask.u32 & (1ul << i)) {
                    zeros = 0;
                } else {
                    if (!zeros) {
                        // incorrect netmask, restoring
                        validateing_object->IP_mask = restored.IP_mask;
                        result |= SV_ERR_NETMASK;
                        break;
                    }
                }
            }
        }

        if (isIPInvalid(validateing_object->IP_gateway) ||
           // is gateway from our subnet?
           ((validateing_object->IP_addr.u32 & validateing_object->IP_mask.u32) !=
            (validateing_object->IP_gateway.u32 & validateing_object->IP_mask.u32))) {
            // incorect gateway, restoring
            validateing_object->IP_gateway = restored.IP_gateway;
            result |= SV_ERR_GATEWAY;
        }

        if (validateing_object->MAC_ADDR[0] & 0b11) {
            // mast be not multycast and global unique
            memcpy(validateing_object->MAC_ADDR, restored.MAC_ADDR, MAC_ADDRESS_SIZE);
            result |= SV_ERR_MAC;
        }

        //----------------------------------------------------------------------

        if ((validateing_object->ReferenceFrequency < REFERENCE_FREQ_MIN) ||
            (validateing_object->ReferenceFrequency > REFERENCE_FREQ_MAX)) {
            validateing_object->ReferenceFrequency = restored.ReferenceFrequency;
            result |= SV_ERR_F_REF;
        }

        if (validateing_object->DHCP > 1) {
            validateing_object->DHCP = restored.DHCP;
            result |= SV_ERR_F_REF;
        }
    }

    settings_update_crc32(validateing_object);
    return result;
}
