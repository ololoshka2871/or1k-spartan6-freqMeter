#ifndef DS1338Z_H
#define DS1338Z_H

#include <sys/time.h>

enum enDS1338z_err {
    DS1338Z_OK = 0,
    DS1338Z_ERROR
};

struct sDS1338z_clock_data {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t DoW;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} __attribute__((packed));

#define DS_1338Z_NVRAM_BASE (sizeof(struct sDS1338z_clock_data) + 1 /*control*/)

enum enDS1338z_err ds1338z_init();

enum enDS1338z_err ds1338z_getRawClockData(struct sDS1338z_clock_data* p);
enum enDS1338z_err ds1338z_getUnixTime(time_t *tm);

enum enDS1338z_err ds1338z_setRawClockData(const struct sDS1338z_clock_data* p);
enum enDS1338z_err ds1338z_setUnixTime(const time_t* tm);

enum enDS1338z_err ds1338z_readNVRAM(void* dest, uint8_t offset, uint8_t size);
enum enDS1338z_err ds1338z_writeNVRAM(uint8_t offset, void* src, uint8_t size);

#endif // DS1338Z_H
