#ifndef DS1338Z_H
#define DS1338Z_H

#include <sys/time.h>

enum enDS1338z_err {
    DS1338Z_OK = 0,
    DS1338Z_ERROR
};

struct sDS1338z_clock_data {
    unsigned            CH:1;
    unsigned            Sec10:3;
    unsigned            Seconds:4;

    unsigned            reserved1:1;
    unsigned            Min10:3;
    unsigned            Min:4;

    unsigned            reserved2:1;
    unsigned            Sel12_24:1;
    unsigned            AM_PM:1;
    unsigned            Hours10:1;
    unsigned            Hours:4;

    unsigned            reserved3:5;
    unsigned            day_of_week:3;

    unsigned            reserved4:2;
    unsigned            Day10:2;
    unsigned            Day:4;

    unsigned            reserved5:3;
    unsigned            Month10;
    unsigned            Month;

    unsigned            Year10:4;
    unsigned            Year:4;

    unsigned            OUT:1;
    unsigned            reserved6:1;
    unsigned            OSF:1;
    unsigned            SQWE:1;
    unsigned            reserved7:2;
    unsigned            RS1:1;
    unsigned            RS0:1;
} __attribute__((packed));

#define NVRAM_BASE          (sizeof(struct sDS1338z_clock_data))

enum enDS1338z_err ds1338z_init();

enum enDS1338z_err ds1338z_getRawClockData(struct sDS1338z_clock_data* p);
enum enDS1338z_err ds1338z_getGetUnixTime(struct tm *tm);

enum enDS1338z_err ds1338z_setRawClockData(const struct sDS1338z_clock_data* p);
enum enDS1338z_err ds1338z_setGetUnixTime(const struct tm *tm);

#endif // DS1338Z_H
