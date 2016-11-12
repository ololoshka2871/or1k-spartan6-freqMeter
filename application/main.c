/****************************************************************************
 * app_main.c
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
#include <stdio.h>
#include <assert.h>

#include "irq.h"
#include "freqmeters.h"
#include "mdio.h"
#include "prog_timer.h"

#include "main.h"
#include "eth-main.h"

#include "eth-dhcp.h"

#include "ETH_config.h"

const char hostname[15] =
#ifdef ETHERNET_HOSTNAME
        ETHERNET_HOSTNAME;
#else
        "SCTB_FM-24_v2";
#endif

static uint16_t measure_time_ms = 10;

static uint32_t timestamps[FREQMETERS_COUNT];
static uint32_t starts[FREQMETERS_COUNT];
static uint32_t measure_time[FREQMETERS_COUNT];
static double Fs[FREQMETERS_COUNT];

static void Process_freqmeters() {
    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        uint32_t ts = fm_getMeasureTimestamp(i);
        if (ts != timestamps[i]) {
            timestamps[i] = ts;
            irq_disable(IS_FREQMETERS);
            uint32_t periods = fm_getActualReloadValue(i);
            uint32_t value   = fm_getActualMeasureTime(i);
            starts[i] = fm_getMeasureStart_pos(i);
            measure_time[i] = fm_getActualReloadValue(i);
            irq_enable(IS_FREQMETERS);
            if ((!value) || (!periods))
                continue;
            double F = (double)periods / (double)value * F_REF;
            Fs[i] = F;
#if 1
            //recalc new reload value
            uint32_t reload_val = (uint32_t)(F * measure_time_ms / 1000);
            if (!reload_val)
                reload_val = 1;
            fm_setChanelReloadValue(i, reload_val, false); // set new reload val
#endif
        }
    }
}

#if 0
static void cb_udp_callback(uint32_t src_ip, uint16_t src_port,
                            uint16_t dst_port, void *data,
                            uint32_t length) {
    assert(data);
#if 1
    struct fm_results {
        uint32_t start;
        uint32_t stop;
        uint32_t mt;
        double res;
    } __attribute__((packed)); // иначе проблемы с выравниванием

    struct fm_results *r = (struct fm_results *)
            allocateUDPpocket(src_ip, sizeof(struct fm_results) * FREQMETERS_COUNT);
    if (!r)
        return;

    for (uint32_t i = 0; i < FREQMETERS_COUNT; ++i) {
        r[i].start = starts[i];
        r[i].stop = timestamps[i];
        r[i].mt = measure_time[i];
        r[i].res = Fs[i];
    }
    sendUDPpacket(src_port, dst_port, r);
#else
#endif 
}
#endif

static void configure_ethernet_PHY() {
    // force 100 Mb/s FD
    MDIO_WriteREG(-1, PHY_BMCR, PHY_BMCR_SPEED100MB | PHY_BMCR_FULL_DUPLEX);

    // set elastic bufer max len
    uint8_t v;
    v = MDIO_ReadREG_sync(-1, PHY_RBR);
    MDIO_WriteREG(-1, PHY_RBR, (v & ~(PHY_RBR_ELAST_BUF_MSK))
                  | PHY_RBR_RMII_REV1_0
                  | (0b00 << PHY_RBR_ELAST_BUF_SH));
}

static void init_tcpip() {
    //----- CONFIGURE ETHERNET -----

#ifdef DHCP_ON_STARTUP
    // dhcp will try to get ip addr
    eth_dhcp_using_manual_settings = 0;
#else
    eth_dhcp_using_manual_settings = 1;
#endif

    eth_dhcp_our_name_pointer = (BYTE*)hostname;

    our_ip_address.v[0] = ETH_IP0; //MSB
    our_ip_address.v[1] = ETH_IP1;
    our_ip_address.v[2] = ETH_IP2;
    our_ip_address.v[3] = ETH_IP3; //LSB
    our_subnet_mask.v[0] = ETH_NETMASK0; //MSB
    our_subnet_mask.v[1] = ETH_NETMASK1;
    our_subnet_mask.v[2] = ETH_NETMASK2;
    our_subnet_mask.v[3] = ETH_NETMASK3; //LSB
    our_gateway_ip_address.v[0] = ETH_GW0;
    our_gateway_ip_address.v[1] = ETH_GW1;
    our_gateway_ip_address.v[2] = ETH_GW2;
    our_gateway_ip_address.v[3] = ETH_GW3;

    //----- SET OUR ETHENET UNIQUE MAC ADDRESS -----
    our_mac_address.v[0] = ETH_MAC0;
    our_mac_address.v[1] = ETH_MAC1;
    our_mac_address.v[2] = ETH_MAC2;
    our_mac_address.v[3] = ETH_MAC3;
    our_mac_address.v[4] = ETH_MAC4;
    our_mac_address.v[5] = ETH_MAC5;

    //----- INITIALISE ETHERNET -----
    tcp_ip_initialise();
}

int main(void)
{
    interrupts_init();
    progtimer_init();
    fm_init();

    for (uint8_t i = 0; i < FREQMETERS_COUNT; ++i) {
        fm_setChanelReloadValue(i, 100, false);
        fm_enableChanel(i, true);
    }

    configure_ethernet_PHY();
    init_tcpip();

    //microip_start(IPTOINT(192, 168, 1, 99));
    //set_rx_callback(cb_udp_callback);

    irq_enable(IS_FREQMETERS);

    EXIT_CRITICAL();

    while(1) {
        //Send_Data();
        Process_freqmeters();
        //microip_service();
        tcp_ip_process_stack();
    }
    return 0;
}
