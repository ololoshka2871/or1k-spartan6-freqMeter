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

#include <stddef.h>

#include "mem_managment.h"
#include "irq.h"

#include "prog_timer.h"

/* Queue of the software timers*/

struct sProgtimer
{
    struct sProgtimer *next;            /* Next timer in list.*/
    progtimer_time_t timer_cnt;         /* Timer counter. */
    progtimer_time_t timer_rv;          /* Timer reference value. */
    void (*handler)(void* cookie);      /* Timer handler. */
    void* cookie;                       /* Handler Cookie. */
};

static struct sProgtimer *progtimer_tl_head = 0;

static volatile progtimer_time_t _current_time;

/************************************************************************
* DESCRIPTION: This function increments current value of the RTC counter.
*************************************************************************/
void progtimer_ticks_inc( void )
{
    _current_time++;
}

/************************************************************************
* DESCRIPTION: Handles timer interrupts
*************************************************************************/
void progtimer_handler_bottom(unsigned int *registers)
{
    (void)registers;

    progtimer_ticks_inc();

    struct sProgtimer *timer = progtimer_tl_head;

    while(timer)
    {
        if(progtimer_get_interval(timer->timer_cnt, _current_time) >= timer->timer_rv)
        {
            timer->timer_cnt = _current_time;

            if(timer->handler)
            {
                timer->handler(timer->cookie);
            }
        }

        timer = timer->next;
    }
}

/************************************************************************
* DESCRIPTION: Starts hardware timer. period 1 ms
*************************************************************************/
void progtimer_init( void )
{
    _current_time = 0u;           /* Reset RTC counter. */

    /* enable HW timer*/
    set_irq_handler(IS_TIMER_SYSTICK, progtimer_handler_bottom);
    irq_enable(IS_TIMER_SYSTICK);
}

/************************************************************************
* DESCRIPTION: Frees the memory, which was allocated for all
*              timers, and disables hardware timer
*************************************************************************/
void progtimer_release( void )
{
    struct sProgtimer *tmp_tl;

    /* disable hw timer */
    irq_disable(IS_TIMER_SYSTICK);

    while(progtimer_tl_head != 0)
    {
        tmp_tl = progtimer_tl_head->next;

        free_sys(progtimer_tl_head);

        progtimer_tl_head = tmp_tl;
    }
}

/************************************************************************
* DESCRIPTION: This function returns current value of the timer in ticks.
*************************************************************************/
progtimer_time_t progtimer_get_ticks( void )
{
    return _current_time;
}

/************************************************************************
* DESCRIPTION: This function returns current value of the timer in seconds.
*************************************************************************/
progtimer_time_t progtimer_get_seconds( void )
{
    return (_current_time / PROGTIMER_TICKS_IN_SEC);
}

/************************************************************************
* DESCRIPTION: This function returns current value of the timer
* in milliseconds.
*************************************************************************/
progtimer_time_t progtimer_get_ms( void )
{
    return (_current_time * PROGTIMER_PERIOD_MS);
}

/************************************************************************
* DESCRIPTION: Creates new software timer with the period
*************************************************************************/
progtimer_desc_t progtimer_new( progtimer_time_t period_ticks, void (*handler)(void* cookie), void* cookie )
{
    struct sProgtimer *timer = NULL;

    if( period_ticks && handler )
    {
        timer = (struct sProgtimer *)malloc_sys(sizeof(struct sProgtimer));

        if(timer)
        {
            timer->next = progtimer_tl_head;

            progtimer_tl_head = timer;

            timer->timer_rv = period_ticks;
            timer->handler = handler;
            timer->cookie = cookie;
        }
    }

    return timer;
}

/************************************************************************
* DESCRIPTION: Frees software timer, which is pointed by tl_ptr
*************************************************************************/
void progtimer_free( progtimer_desc_t timer )
{
    struct sProgtimer *tl = timer;
    struct sProgtimer *tl_temp;

    if(tl)
    {
        if(tl == progtimer_tl_head)
        {
            progtimer_tl_head = progtimer_tl_head->next;
        }
        else
        {
            tl_temp = progtimer_tl_head;

            while(tl_temp->next != tl)
            {
                tl_temp = tl_temp->next;
            }

            tl_temp->next = tl->next;
        }

        free_sys(tl);
    }
}

/************************************************************************
* DESCRIPTION: Resets all timers' counters
*************************************************************************/
void progtimer_reset_all( void )
{
    struct sProgtimer *tl;

    tl = progtimer_tl_head;

    while(tl != 0)
    {
        tl->timer_cnt = _current_time;
        tl = tl->next;
    }
}

/************************************************************************
* DESCRIPTION: Calaculates an interval between two moments of time
*************************************************************************/
progtimer_time_t progtimer_get_interval( progtimer_time_t start, progtimer_time_t end )
{
    if(start <= end)
    {
        return (end - start);
    }
    else
    {
        return (0xffffffffu - start + end + 1u);
    }
}

/************************************************************************
* DESCRIPTION: Do delay for a given number of timer ticks.
*************************************************************************/
void progtimer_delay( progtimer_time_t delay_ticks )
{
    progtimer_time_t start_ticks = _current_time;

    while(progtimer_get_interval(start_ticks, progtimer_get_ticks()) < delay_ticks);
}

/************************************************************************
* DESCRIPTION: Convert milliseconds to timer ticks.
*************************************************************************/
progtimer_time_t progtimer_ms2ticks( progtimer_time_t time_ms )
{
    return time_ms / PROGTIMER_PERIOD_MS;
}

/************************************************************************
* DESCRIPTION: Set curent time ticks
*************************************************************************/
void progtimer_setclock( progtimer_time_t time_ms )
{
    ENTER_CRITICAL();
    _current_time = time_ms / PROGTIMER_PERIOD_MS;
    progtimer_reset_all();
    EXIT_CRITICAL();
}

void progtimer_set_timer_interval(progtimer_desc_t timer, progtimer_time_t interval) {
    timer->timer_rv = interval;
}

void progtimer_set_timer_counter(progtimer_desc_t timer, progtimer_time_t pos) {
    timer->timer_cnt = pos;
}

progtimer_time_t progtimer_get_timer_counter(progtimer_desc_t timer) {
    return progtimer_get_interval(_current_time, timer->timer_cnt);
}

progtimer_time_t progtimer_get_timer_interval(progtimer_desc_t timer) {
    return timer->timer_rv;
}
