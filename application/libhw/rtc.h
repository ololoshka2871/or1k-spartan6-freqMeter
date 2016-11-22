#ifndef RTC_H
#define RTC_H

#include <sys/time.h>

void rtc_init();
int clock_gettime(clockid_t clockid, struct tm *tm);
int clock_settime(clockid_t clockid, const struct tm *tm);

#endif // RTC_H
