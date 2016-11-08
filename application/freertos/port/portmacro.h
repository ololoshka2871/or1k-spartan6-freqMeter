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


#ifndef _PORTMACRO_H_
#define _PORTMACRO_H_

#include "irq.h"

// http://opencores.org/or1k/FreeRTOS

#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  unsigned portLONG
#define portBASE_TYPE   long
#define portTickType    unsigned portLONG
#define portMAX_DELAY   (portTickType)0xffffffff

#define portSTACK_GROWTH                -1
#define portTICK_RATE_MS                ( (portTickType) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT              4
#define portCRITICAL_NESTING_IN_TCB     1
#define portINSTRUCTION_SIZE            ( ( portSTACK_TYPE ) 4 )
#define portNO_CRITICAL_SECTION_NESTING ( ( portSTACK_TYPE ) 0 )
#define portYIELD_FROM_ISR()            vTaskSwitchContext()
#ifdef __OR1K_NODELAY__
#define portYIELD()                     __asm__ __volatile__ ("l.sys 0x0FCC")
#else
#define portYIELD()                     do {\
    __asm__ __volatile__ ( "l.sys 0x0FCC" ); \
    __asm__ __volatile__ ( "l.nop       " ); \
} while (0)
#endif
#define portSysCall_yeld            0x0fcc

#define portNOP()                  __asm__ __volatile__ ( "l.nop" )
#define portDISABLE_INTERRUPTS()   __or1k_disable_interrupts()
#define portENABLE_INTERRUPTS()    __or1k_enable_interrupts()

#define portENTER_CRITICAL()       __or1k_disable_interrupts()
#define portEXIT_CRITICAL()        __or1k_enable_interrupts()

#ifdef __OR1K_NODELAY__
#define portRESTORE_CONTEXT()      __asm__ __volatile__ ("l.sys 0x0FCD")
#else
#define portRESTORE_CONTEXT()       do { \\
    __asm__ __volatile__ ( "l.sys 0x0FCD" ); \
    __asm__ __volatile__ ( "l.nop       " ); \
} while(0)
#endif
#define portSysCall_restoreContext  0x0fcD

#if 0
#ifdef __OR1K_NODELAY__
#define portSAVE_CONTEXT()         __asm__ __volatile__ ("l.sys 0x0FCE")
#else
#define portSAVE_CONTEXT()       do { \
    __asm__ __volatile__ ( "l.sys 0x0FCE" ); \
    __asm__ __volatile__ ( "l.nop       " ); \
} while(0)
#endif
#define portSysCall_saveContext     0x0fcE
#endif

typedef portBASE_TYPE   BaseType_t;
typedef portSTACK_TYPE  StackType_t;
typedef portTickType    TickType_t;
typedef unsigned  portBASE_TYPE UBaseType_t;

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#endif /* _PORTMACRO_H_ */
