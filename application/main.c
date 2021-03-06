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
#include <stdio.h>
#include <assert.h>

#include "irq.h"
#include "freqmeters.h"
#include "mdio.h"
#include "prog_timer.h"
#include "rtc.h"
#include "GPIO.h"
#include "prog_timer.h"
#include "settings.h"

#include "main.h"
#include "eth-dhcp.h"
#include "udp-protobuf_server.h"
#include "websoc_server.h"

const char hostname[15] =
#ifdef ETHERNET_HOSTNAME
        ETHERNET_HOSTNAME;
#else
        "SCTB_FM-24_v2";
#endif


#define FORCE_STATIC_IP_PIN         (1 << 3)
#define IS_STATIC_IP_FORCED()       (~gpio_port_get_val(GPIO_PORTA) & FORCE_STATIC_IP_PIN)

static void configure_ethernet_PHY() {
    volatile uint8_t v;
    // force 100 Mb/s FD
    MDIO_WriteREG(-1, PHY_BMCR, PHY_BMCR_SPEED100MB | PHY_BMCR_FULL_DUPLEX);
    v = MDIO_ReadREG_sync(-1, PHY_BMCR);
    // set elastic bufer max len
    v = MDIO_ReadREG_sync(-1, PHY_RBR);
    MDIO_WriteREG(-1, PHY_RBR, (v & ~(PHY_RBR_ELAST_BUF_MSK))
                  | PHY_RBR_RMII_REV1_0
                  | (0b00 << PHY_RBR_ELAST_BUF_SH));
    v = MDIO_ReadREG_sync(-1, PHY_RBR);
}

static void init_tcpip() {
    //----- CONFIGURE ETHERNET -----
    eth_dhcp_our_name_pointer = (BYTE*)hostname;

    if (settings.DHCP
#if GPIO_ENABLED
        && !IS_STATIC_IP_FORCED()
#endif
        ) {
        eth_dhcp_using_manual_settings = 0;
    } else {
        eth_dhcp_using_manual_settings = 1;
    }

    // ---- IP Addr settings
    memcpy(our_ip_address.v, settings.IP_addr.u8, sizeof(union IP_ADDR));
    memcpy(our_subnet_mask.v, settings.IP_mask.u8, sizeof(union IP_ADDR));
    memcpy(our_gateway_ip_address.v, settings.IP_gateway.u8, sizeof(union IP_ADDR));

    //----- SET OUR ETHENET UNIQUE MAC ADDRESS -----
    memcpy(our_mac_address.v, settings.MAC_ADDR, sizeof(MAC_ADDR));

    //----- INITIALISE ETHERNET -----
    tcp_ip_initialise();
}

#define LED_MASK    (0b111)

static void led_blinker(void* cookie) {
    (void)cookie;
    uint32_t v = gpio_port_get_val(GPIO_PORTA) & LED_MASK;
    uint32_t new_v = (v & (LED_MASK >> 1)) ? (v << 1) : 1;

    gpio_port_set_val(GPIO_PORTA, new_v, v);
}

static void Led_toggle() {
#if GPIO_ENABLED
    uint32_t v = gpio_port_get_val(GPIO_PORTA) & 1;
    uint32_t set = (~v) & 1;

    gpio_port_set_val(GPIO_PORTA, set, v);
#endif
}

static void initAll() {
    interrupts_init();
    progtimer_init();
    fm_init();

#if GPIO_ENABLED
    gpio_port_init(GPIO_PORTA, LED_MASK);
    progtimer_new(1000, led_blinker, NULL);
#endif

    rtc_init();
    Settings_init();

    configure_ethernet_PHY();
    init_tcpip();
}

int main(void)
{
    initAll();
    EXIT_CRITICAL();

    while(1) {
        fm_process();
        tcp_ip_process_stack();

#ifdef PROCESS_SERVER_UDP
        process_protobuf_server(NULL);
#endif

#ifdef PROCESS_SERVER_WEBSOC
        process_websoc_server();
#endif
    }
    return 0;
}
