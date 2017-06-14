#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

union IP_ADDR {
    uint8_t u8[4];
    uint32_t u32;
};

enum enSettingsValidatorError {
    SV_ERR_OK = 0,
    SV_ERR_IP = 1 << 0,
    SV_ERR_NETMASK = 1 << 1,
    SV_ERR_GATEWAY = 1 << 2,
    SV_ERR_MAC = 1 << 3,
    SV_ERR_F_REF = 1 << 4,
    SV_ERR_CRC = 1 << 7,
};

#define MAC_ADDRESS_SIZE    6

// max size 55 bytes
struct sSettings {
    // -- net 19 bytes
    union IP_ADDR IP_addr;
    union IP_ADDR IP_mask;
    union IP_ADDR IP_gateway;
    uint8_t MAC_ADDR[MAC_ADDRESS_SIZE];
    uint8_t DHCP;

    uint32_t ReferenceFrequency;

    uint32_t CRC32; // 4 bytes
};


typedef void (*validate_restorer)(struct sSettings* settings);

// инициализация
void Settings_init();

// записать в NVRAM из settings ни чего не проверяется
void Settings_write(struct sSettings* settings);
// прочитать из NVRAM, положить в settings ни чего не проверяется
void Settings_read(struct sSettings *settings);

// валидация полей, использует указанный колбэк, чтобы привезти данные к валидному виду
// чтение сохраненых == restore или сброс на дефолтные == reset
// true - ok, false - restored
enum enSettingsValidatorError
Settings_validate(struct sSettings* validateing_object, validate_restorer restorer);

void settings_update_crc32(struct sSettings *settings);

void Settings_defaults(struct sSettings *settings);

extern struct sSettings settings;

#endif // SETTINGS_H
