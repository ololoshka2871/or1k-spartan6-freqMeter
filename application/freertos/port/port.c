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

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "projdefs.h"
#include "syscall.h"
#include "irq.h"

struct context {
    portSTACK_TYPE r0;
    portSTACK_TYPE r1;
    portSTACK_TYPE r2;
    portSTACK_TYPE r3;
    portSTACK_TYPE r4;
    portSTACK_TYPE r5;
    portSTACK_TYPE r6;
    portSTACK_TYPE r7;
    portSTACK_TYPE r8;
    portSTACK_TYPE r9;
    portSTACK_TYPE r10;
    portSTACK_TYPE r11;
    portSTACK_TYPE r12;
    portSTACK_TYPE r13;
    portSTACK_TYPE r14;
    portSTACK_TYPE r15;
    portSTACK_TYPE r16;
    portSTACK_TYPE r17;
    portSTACK_TYPE r18;
    portSTACK_TYPE r19;
    portSTACK_TYPE r20;
    portSTACK_TYPE r21;
    portSTACK_TYPE r22;
    portSTACK_TYPE r23;
    portSTACK_TYPE r24;
    portSTACK_TYPE r25;
    portSTACK_TYPE r26;
    portSTACK_TYPE r27;
    portSTACK_TYPE r28;
    portSTACK_TYPE r29;
    portSTACK_TYPE r30;
    portSTACK_TYPE r31;
    portSTACK_TYPE X;
    portSTACK_TYPE EPC;
    portSTACK_TYPE ESR;
};


extern struct context * volatile pxCurrentTCB; // current task context

static unsigned int* switch_context(unsigned int* current_context) {
    memcpy(pxCurrentTCB, current_context, sizeof(struct context)); // save curent context to pxCurrentTCB
    vTaskSwitchContext(); // update pxCurrentTCB
    return (unsigned int *)pxCurrentTCB; // return new tcb
}

// syscall context
static unsigned int*  vSystemCallHandler(unsigned int code,
                                         unsigned int *registers) {
    switch (code) {
    case portSysCall_yeld:
        registers = switch_context(registers); // restore context from pxCurrentTCB
        break;
    case portSysCall_restoreContext:
        registers = (unsigned int *)pxCurrentTCB;
        break;
    default:
        break;
    }

    return registers;
}

// interrupt context
static unsigned int * vSysTimerTick(unsigned int *registers) {
    if( xTaskIncrementTick() != pdFALSE ) {
        return switch_context(registers);
    }
    return NULL;
}


StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters ) {

    struct context *xTopOfStack = (struct context *) pxTopOfStack;

    xTopOfStack->r0 = (portSTACK_TYPE ) 0;           /* R0 = 0 */
    xTopOfStack->r1 = (portSTACK_TYPE ) xTopOfStack; /* R1 (SP) */
    xTopOfStack->r2 = (portSTACK_TYPE ) 2;           /* R2 */
    xTopOfStack->r3 = (portSTACK_TYPE ) pvParameters;/* R3 (parameter 0) */
    xTopOfStack->r4 = (portSTACK_TYPE ) 4;           /* R4 */
    xTopOfStack->r5 = (portSTACK_TYPE ) 5;           /* R5 */
    xTopOfStack->r6 = (portSTACK_TYPE ) 6;           /* R6 */
    xTopOfStack->r7 = (portSTACK_TYPE ) 7;           /* R7 */
    xTopOfStack->r8 = (portSTACK_TYPE ) 8;           /* R8 */
    xTopOfStack->r9 = (portSTACK_TYPE ) 9;           /* R9 (LR) */
    xTopOfStack->r10 = (portSTACK_TYPE ) 10;         /* R10 */
    xTopOfStack->r11 = (portSTACK_TYPE ) 11;         /* R11 */
    xTopOfStack->r12 = (portSTACK_TYPE ) 12;         /* R12 */
    xTopOfStack->r13 = (portSTACK_TYPE ) 13;         /* R13 */
    xTopOfStack->r14 = (portSTACK_TYPE ) 14;         /* R14 */
    xTopOfStack->r15 = (portSTACK_TYPE ) 15;         /* R15 */
    xTopOfStack->r16 = (portSTACK_TYPE ) 16;         /* R16 */
    xTopOfStack->r17 = (portSTACK_TYPE ) 17;         /* R17 */
    xTopOfStack->r18 = (portSTACK_TYPE ) 18;         /* R18 */
    xTopOfStack->r19 = (portSTACK_TYPE ) 19;         /* R19 */
    xTopOfStack->r20 = (portSTACK_TYPE ) 20;         /* R20 */
    xTopOfStack->r21 = (portSTACK_TYPE ) 21;         /* R21 */
    xTopOfStack->r22 = (portSTACK_TYPE ) 22;         /* R22 */
    xTopOfStack->r23 = (portSTACK_TYPE ) 23;         /* R23 */
    xTopOfStack->r24 = (portSTACK_TYPE ) 24;         /* R24 */
    xTopOfStack->r25 = (portSTACK_TYPE ) 25;         /* R25 */
    xTopOfStack->r26 = (portSTACK_TYPE ) 26;         /* R26 */
    xTopOfStack->r27 = (portSTACK_TYPE ) 27;         /* R27 */
    xTopOfStack->r28 = (portSTACK_TYPE ) 28;         /* R28 */
    xTopOfStack->r29 = (portSTACK_TYPE ) 29;         /* R29 */
    xTopOfStack->r30 = (portSTACK_TYPE ) 30;         /* R30 */
    xTopOfStack->r31 = (portSTACK_TYPE ) 31;         /* R31 */
    xTopOfStack->X   = (portSTACK_TYPE ) 0;          /* R31 */
    xTopOfStack->EPC = (portSTACK_TYPE ) pxCode;     /* PC */
    xTopOfStack->ESR = (portSTACK_TYPE ) mfspr(64);  /* SR */

    /* Return a pointer to the top of the stack we have generated so this can
     be stored in the task control block for the task. */
    return pxTopOfStack;
}

static void prvSetupTimerInterrupt() {
    set_irq_handler(IS_TIMER_SYSTICK, vSysTimerTick);
    setInterruptPriority(IS_TIMER_SYSTICK, IRQ_LOWEST_PRIO);
    irq_enable(IS_TIMER_SYSTICK);
}

BaseType_t xPortStartScheduler( void ) {
    /* Setup the hardware to generate the tick.  Interrupts are disabled when
     this function is called. */
    prvSetupTimerInterrupt();

    /* Install interrupt handler for system call exception, this will allow
     us to yield manually */
    install_syscall_handler(vSystemCallHandler);

    /* Restore the context of the first task that is going to run. */
    portRESTORE_CONTEXT();

    /* Should not get here as the tasks are now running! */
    return pdTRUE;
}
