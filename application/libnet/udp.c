/*
 * This file is part of m-labs/misoc project
 * https://github.com/m-labs/misoc
 * Modified by Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 */

#include <stddef.h>

#include "microip.h"

#include "udp.h"

static udp_callback rx_callback = NULL;

void process_udp(struct udp_frame* rx_udp)
{
    if(rx_udp->ip.TotalLength < sizeof(struct udp_frame)) return;
    if(rx_udp->udp.length < sizeof(struct udp_header)) return;
    if(rx_callback)
        rx_callback(rx_udp->ip.SourceIP, rx_udp->udp.src_port,
                    rx_udp->udp.dst_port, rx_udp->payload,
                    rx_udp->udp.length-sizeof(struct udp_header));
}
