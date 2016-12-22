/****************************************************************************
 * src/main.c
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

#include "syscall.h"

uint32_t __attribute__((noinline)) syscall(uint32_t arg) {
    // arg == r3 -> first argument for syscall
    asm volatile("l.sys 8"); // 8 == exec user's syscall
    return arg;
}

syscall_handler __attribute__((noinline))
install_syscall_handler(syscall_handler handler) {
    // handler == r3
    asm volatile("l.sys 4"); // r3 now old syscall handler
    return handler;
}

void __attribute__((noinline))
read_boot_flash(uint32_t addr, uint8_t *dest, uint32_t size) {
    // addr == r3 -> addr to read from
    // desr == r4 -> deftination buffer
    // size == r5 -> size to read
    asm volatile("l.sys 6"
                 : "=r" (addr)
                 : "r" (addr), "r" (dest), "r" (size)
                 ); // 6 == read boot flash
}

void reboot() {
    asm volatile("l.sys 7"); // 7 == reboot
}

//-----------------------------------------------------------------
// mfspr: Read from SPR
//-----------------------------------------------------------------
unsigned long mfspr(unsigned long spr)
{
    unsigned long value;
    asm volatile ("l.mfspr\t\t%0,%1,0" : "=r" (value) : "r" (spr));
    return value;
}
//-----------------------------------------------------------------
// mtspr: Write to SPR
//-----------------------------------------------------------------
void mtspr(unsigned long spr, unsigned long value)
{
    asm volatile ("l.mtspr\t\t%0,%1,0": : "r" (spr), "r" (value));
}
