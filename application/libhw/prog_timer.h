#ifndef _PROG_TIMER_H
#define _PROG_TIMER_H

#include <stdint.h>

#define PROGTIMER_PERIOD_MS        (1U) /* Do not change it.*/
#define PROGTIMER_TICKS_IN_HOUR    ((1000U*60U*60U)/PROGTIMER_PERIOD_MS)
#define PROGTIMER_TICKS_IN_MIN     ((1000U*60U)/PROGTIMER_PERIOD_MS)
#define PROGTIMER_TICKS_IN_SEC     (1000U/PROGTIMER_PERIOD_MS)

#if defined(__cplusplus)
extern "C" {
#endif

typedef uint64_t progtimer_time_t;

typedef struct sProgtimer* progtimer_desc_t;

void progtimer_init( void );
progtimer_time_t progtimer_get_ticks( void );
progtimer_time_t progtimer_get_seconds( void );
progtimer_time_t progtimer_get_ms( void );
progtimer_time_t progtimer_ms2ticks( progtimer_time_t time_ms );
progtimer_time_t progtimer_get_interval( progtimer_time_t start, progtimer_time_t end );
progtimer_desc_t progtimer_new( progtimer_time_t period_ticks, void (*handler)(void* cookie), void* cookie );
void progtimer_delay( progtimer_time_t delay_ticks );
void progtimer_reset_all( void );
void progtimer_setclock( progtimer_time_t time_ms );
void progtimer_set_timer_interval(progtimer_desc_t timer, progtimer_time_t interval);
void progtimer_set_timer_counter(progtimer_desc_t timer, progtimer_time_t pos);
progtimer_time_t progtimer_get_timer_counter(progtimer_desc_t timer);
progtimer_time_t progtimer_get_timer_interval(progtimer_desc_t timer);

#if defined(__cplusplus)
}
#endif

#endif /* _PROG_TIMER_H */
