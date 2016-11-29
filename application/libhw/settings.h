#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>

union IP_ADDR {
    uint8_t u8[4];
    uint32_t u32;
};

// max size 55 bytes
struct sSettings {
    // -- net 19 bytes
    union IP_ADDR IP_addr;
    union IP_ADDR IP_mask;
    union IP_ADDR IP_gateway;
    uint8_t MAC_ADDR[6];
    uint8_t DHCP;

    uint32_t CRC32; // 4 bytes
};

void Settings_init();
void Settings_validate();
void Settings_read();
void Settings_write();

extern struct sSettings settings;

#endif // SETTINGS_H
