#include "mem_map.h"
#include "timer.h"

//--------------------------------------------------------------------------
// timer_init:
//--------------------------------------------------------------------------
void timer_init(void)
{
    // Not required
}
//--------------------------------------------------------------------------
// timer_sleep:
//--------------------------------------------------------------------------
void timer_sleep(int timeMs)
{
    t_time t = timer_now();

    while (timer_diff(timer_now(), t) < timeMs)
        ;
}

void hires_timer_sleep(int ticks)
{
    t_time t = hires_timer_now();

    while (timer_diff(hires_timer_now(), t) < ticks);
}

t_time hires_timer_now() { return SYS_CLK_COUNT; }
