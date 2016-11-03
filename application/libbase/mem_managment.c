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

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>

#include "mem_managment.h"

typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
    size_t xBlockSize;						/*<< The size of the free block. */
} BlockLink_t;

struct sHeapPools {
    BlockLink_t xStart;
    BlockLink_t *pxEnd;
    size_t xFreeBytesRemaining;
    size_t xMinimumEverFreeBytesRemaining;
    size_t xBlockAllocatedBit;
    size_t heap_base;
    size_t configTOTAL_HEAP_SIZE;
};

#ifndef SYSTEM_HEAP_SIZE
#error "SYSTEM_HEAP_SIZE mast be defined"
#endif

#ifndef MACTX_HEAP_BASE
#error "MACTX_HEAP_BASE mast be defined"
#endif

#ifndef MACTX_HEAP_SIZE
#error "MACTX_HEAP_SIZE mast be defined"
#endif

static volatile uint8_t heap[ SYSTEM_HEAP_SIZE ] __attribute__((section("system.heap")));

struct sHeapPools heap_table[] = {
    { // system
        {NULL, 0}, NULL, 0, 0, 0, (size_t)heap, sizeof(heap)
    },
    { // eth_tx
        {NULL, 0}, NULL, 0, 0, 0, MACTX_HEAP_BASE, MACTX_HEAP_SIZE
    }
};

#define _xStart(pool)                   (heap_table[(pool)].xStart)
#define _pxEnd(pool)                    (heap_table[(pool)].pxEnd)
#define _xFreeBytesRemaining(pool)      (heap_table[(pool)].xFreeBytesRemaining)
#define _xMinimumEverFreeBytesRemaining(pool)      (heap_table[(pool)].xMinimumEverFreeBytesRemaining)
#define _xBlockAllocatedBit(pool)       (heap_table[(pool)].xBlockAllocatedBit)
#define _ucHeap(pool)                   (heap_table[(pool)].heap_base)
#define configTOTAL_HEAP_SIZE(pool)     (heap_table[(pool)].configTOTAL_HEAP_SIZE)
#define configASSERT(v)                 assert(v);
#define mtCOVERAGE_TEST_MARKER()


// define hooks for FREERTOS memory manager

#define configSUPPORT_DYNAMIC_ALLOCATION    (1)
#define configAPPLICATION_ALLOCATED_HEAP    (1)
#define configUSE_MALLOC_FAILED_HOOK        (0)

#define vTaskSuspendAll()
#define traceMALLOC(a, b)
#define traceFREE(a, b)
#define xTaskResumeAll()                    (0)
#define portBYTE_ALIGNMENT                  (4)
#define portBYTE_ALIGNMENT_MASK             (0x0003)

void *pvPortZalloc( enum enHeapPools pool, size_t xWantedSize ) {
    void *r = pvPortMalloc( pool, xWantedSize );
    if (r)
        memset(r, 0, xWantedSize);
    return r;
}


#include "heap_4.c"
