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
#include <string.h>

#include "mem_map.h"
#include "syscall.h"

#include "irq.h"

struct src_handler {
    enum InterruptSources src;
    isr_handler ISR;
};

static struct src_handler ISRs[IS_Count] = {
{ IS_TIMER_HIRES, NULL },
{ IS_FREQMETERS, NULL },
{ IS_TIMER_SYSTICK, NULL },
{ IS_UART0, NULL },
{ IS_BOOT_SPI, NULL },
{ IS_GPIO, NULL },
{ IS_MDIO, NULL },
{ IS_I2C, NULL },
{ IS_MINIMAC_TX, NULL },
{ IS_MINIMAC_RX, NULL },
};

static unsigned int irq_disable_counter = 1;

//-----------------------------------------------------------------
// irq_enable 
//-----------------------------------------------------------------
void irq_enable(int interrupt)
{
    IRQ_MASK_SET = (1 << interrupt);
}
//-----------------------------------------------------------------
// irq_disable 
//-----------------------------------------------------------------
void irq_disable(int interrupt)
{
    IRQ_MASK_CLR = (1 << interrupt);
}
//-----------------------------------------------------------------
// irq_acknowledge 
//-----------------------------------------------------------------
void irq_acknowledge(int interrupt)
{
    IRQ_STATUS = (1 << interrupt);
}
//-----------------------------------------------------------------
// irq_check
//-----------------------------------------------------------------
int irq_check(int interrupt)
{
    return IRQ_STATUS & (1 << interrupt);
}


//-----------------------------------------------------------------

static unsigned int* default_ISR(unsigned int * registers) {
    // check irq source
    for (int i = 0; i < IS_Count; ++i) {
        uint32_t interrup_flags = IRQ_STATUS & IRQ_MASK;
        struct src_handler* d = &ISRs[i];
        if (interrup_flags & (1 << d->src) ) {
            if (d->ISR)
                d->ISR(registers);
            irq_acknowledge(d->src);
            // Акноледж всегда после тела прерывания, иначе может случиться
            // следующая ситуация:
            // прерывание на частотомере появилось, мы зашли сюда и сдедаи
            // акноледж. Однако каналы еще не перезагружены и флаг прерывания
            // появится на следующем-же такте и весь обработчик вызовется вновь,
            // а обработчик частотомера получит флаги 0 во всех каналах, ибо
            // он только-что их обработал, а новых еще не могло успеть появиться.

            interrup_flags &= ~(1 << d->src);
            if (!interrup_flags)
                break;
        }
    }

    return registers;
}

void setInterruptPriority(enum InterruptSources src, uint8_t new_prio) {
    int current_prio = -1;
    if (new_prio >= IS_Count)
        return;
    // find current prio
    for (int i = 0; i < IS_Count; ++i)
        if (ISRs[i].src == src)
            current_prio = i;
    if(current_prio == -1)
        return;

    if (current_prio == new_prio)
        return;

    struct src_handler _ISRs[IS_Count];
    memcpy(_ISRs, ISRs, sizeof(ISRs));
    isr_handler isr_fun = ISRs[current_prio].ISR;
    if (current_prio > new_prio) {
        // shift fragment up

        // 1 ----                           1 ----
        // 2 current_prio                   3 ----
        // 3 ----                           4 ----
        // 4 ----                           5 ----
        // 5 ---- <- prio                   2 prio
        // 6 ----                           6 ----
        memmove(&_ISRs[current_prio],
                &_ISRs[current_prio + 1],
                sizeof(struct src_handler) * (current_prio - new_prio));
    } else {
        // shift fragment down

        // 1 ----                           1 ----
        // 2 ---- <- prio                   5 prio
        // 3 ----                           2 ----
        // 4 ----                           3 ----
        // 5 current_prio                   4 ----
        // 6 ----                           6 ----
        memmove(&_ISRs[new_prio + 1],
                &_ISRs[new_prio],
                sizeof(struct src_handler) * (new_prio - current_prio));
    }
    _ISRs[new_prio].src = src;
    _ISRs[new_prio].ISR = isr_fun;
    ENTER_CRITICAL();
    memcpy(ISRs, _ISRs, sizeof(ISRs));
    EXIT_CRITICAL();
}


isr_handler set_irq_handler(enum InterruptSources src, isr_handler handler) {
    for (int i = 0; i < IS_Count; ++i) {
        struct src_handler* d = &ISRs[i];
        if (d->src == src) {
            isr_handler result = d->ISR;
            ENTER_CRITICAL();
            d->ISR = handler;
            EXIT_CRITICAL();
            return result;
        }
    }
    return NULL;
}

irq_handler __attribute__((noinline)) install_irq_global_handler(irq_handler handler) {
    asm volatile("l.sys 5");
    return handler; // old handler value from syscall
}

void interrupts_init(void) {
    install_irq_global_handler(default_ISR);
}


void __or1k_disable_interrupts(void) {
    ++irq_disable_counter;
#if 1
    unsigned long sr = mfspr(SPR_SR);
    mtspr(SPR_SR, sr & ~SPR_SR_GIE);
#else
    unsigned long value;
    asm volatile ("l.mfspr\t\t%0,r0,%1" : "=r" (value) : "i" (SPR_SR));
    asm volatile ("l.mtspr\t\tr0,%0,%1" :
                  : "r" (value & ~SPR_SR_GIE), "i" (SPR_SR));
#endif
}

void __or1k_enable_interrupts(void) {
    if (irq_disable_counter) {
        --irq_disable_counter;
        if (irq_disable_counter)
            return;
    }
#if 1
    unsigned long sr = mfspr(SPR_SR);
    mtspr(SPR_SR, sr | SPR_SR_GIE);
#else
    unsigned long value;
    asm volatile ("l.mfspr\t\t%0,r0,%1" : "=r" (value) : "i" (SPR_SR));
    asm volatile ("l.mtspr\t\tr0,%0,%1" :
                  : "r" (value | SPR_SR_GIE), "i" (SPR_SR));
#endif
}
