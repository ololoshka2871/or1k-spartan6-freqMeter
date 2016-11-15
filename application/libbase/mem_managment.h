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

#ifndef _HEAP_H_
#define _HEAP_H_

#include <stddef.h>

enum enHeapPools {
    HEAP_POOL_SYSTEM = 0,
    HEAP_POOL_ETH_TX = 1
};

#define malloc_sys(size)        pvPortMalloc(HEAP_POOL_SYSTEM, size)
#define zalloc_sys(size)        pvPortZalloc(HEAP_POOL_SYSTEM, size)
#define free_sys(p)             vPortFree(HEAP_POOL_SYSTEM, p)
#define get_heap_free_sys()     xPortGetFreeHeapSize(HEAP_POOL_SYSTEM)

#define malloc_mac_tx(size)     pvPortMalloc(HEAP_POOL_ETH_TX, size)
#define free_mac_tx(p)          vPortFree(HEAP_POOL_ETH_TX, p)
#define get_heap_free_mac_tx()  xPortGetFreeHeapSize(HEAP_POOL_ETH_TX)

void *pvPortMalloc( enum enHeapPools pool, size_t xWantedSize );
void *pvPortZalloc( enum enHeapPools pool, size_t xWantedSize );
void vPortFree( enum enHeapPools pool, void *pv );
size_t xPortGetFreeHeapSize( enum enHeapPools pool );

#endif /*_HEAP_H_*/
