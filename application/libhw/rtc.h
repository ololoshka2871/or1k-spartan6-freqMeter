#ifndef RTC_H
#define RTC_H

#include <sys/time.h>

void rtc_init();
int clock_gettime(clockid_t clockid, struct timespec *ts);
int clock_settime(clockid_t clockid, uint64_t ts);
uint64_t clock_catch_timestamp();

#endif // RTC_H
