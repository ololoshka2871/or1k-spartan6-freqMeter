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

#include "main.h"					//Global data type definitions (see https://github.com/ibexuk/C_Generic_Header_File )
#include "eth-main.h"		//Include before our header file
#define	NIC_C
#include "eth-nic.h"

#ifdef STACK_USE_DHCP
#include "eth-dhcp.h"
#endif

#include "mdio.h"

#include "prog_timer.h"

#ifdef STACK_USE_DHCP
static void tick_1ms(void* p)  {
    (void)p;

    //----- NIC DHCP TIMER -----
    if (eth_dhcp_1ms_timer)
        eth_dhcp_1ms_timer--;
}
#endif

static void tick_1s(void* p)  {
    (void)p;
#ifdef STACK_USE_DHCP
    if (eth_dhcp_1sec_renewal_timer)
        eth_dhcp_1sec_renewal_timer--;
#endif
    if (eth_dhcp_1sec_lease_timer)
        eth_dhcp_1sec_lease_timer--;

    // check PHY connection
    nic_is_linked = MDIO_getConnectionStatus(-1);
}


static void tick_10ms(void* p)  {
    (void)p;

    //----- ETHERNET GENERAL TIMER -----
    ethernet_10ms_clock_timer_working++;
}

void init_eth_timers() {
    // save no pointer, can't free
#ifdef STACK_USE_DHCP
    progtimer_new(1, tick_1ms, NULL);
#endif
    progtimer_new(10, tick_10ms, NULL);
    progtimer_new(1000, tick_1s, NULL);
}
