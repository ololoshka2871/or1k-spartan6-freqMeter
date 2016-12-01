#ifndef RTC_H
#define RTC_H

#include <sys/time.h>

void rtc_init();
int clock_gettime(clockid_t clockid, struct timespec *ts);
int clock_settime(clockid_t clockid, const struct timespec *ts);
void clock_catch_inpure_timestamp(struct timespec *ts);
void clock_purify_time(struct timespec *ts);

#endif // RTC_H
