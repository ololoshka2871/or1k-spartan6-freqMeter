/*
 * This file is part of m-labs/misoc project
 * https://github.com/m-labs/misoc
 * Modified by Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "minimac.h"

#ifdef STATS_PORT
#include <stdio.h>
#include "microip.h"
#endif

#include "udp.h"

static udp_callback rx_callback = NULL;

void process_udp(struct udp_frame* rx_udp)
{
    if(rx_udp->udp.length < sizeof(struct udp_header)) return;
    // ignore checksumm

#ifndef STATS_PORT
    if(rx_callback)
        rx_callback(rx_udp->ip.SourceIP, rx_udp->udp.src_port,
                    rx_udp->udp.dst_port, rx_udp->payload,
                    rx_udp->udp.length-sizeof(struct udp_header));
#else
    char buff[256];
    struct sminiMAC_Stat stat;
    minMAC_stat(&stat);
    sprintf(buff, "rx: %d\ttx: %d\trx_drop: %d\tLE: %d\tpyload_size: %d\n", stat.pocket_rx,
            stat.pocket_tx, stat.pocket_rx_errors, stat.last_error, rx_udp->udp.length - sizeof(struct udp_header));

    send_udp_packet(rx_udp->ip.SourceIP, STATS_PORT, rx_udp->udp.dst_port,
                     buff, strlen(buff));
#endif
}

void set_rx_callback(udp_callback callback) {
    rx_callback = callback;
}

uint8_t *allocateUDPpocket(uint32_t ip_to, size_t data_len) {
    struct udp_pocket * udp_packet;
    size_t pocket_len = data_len + sizeof(struct udp_header);
    struct ip_header* ip_packet =
            microip_allocate_ip_pocket((uint8_t**)&udp_packet, ip_to, pocket_len);
    if (!ip_packet) {
        return NULL;
    }
    udp_packet->hader.length = pocket_len;
    udp_packet->hader.checksum = 0; // checksumm not used

    // без этого почему-то возвращается udp_packet
    uint8_t * res = (uint8_t*)&udp_packet->payload[0];
    return res;
}

/// @data - preallocated by alocateUDPpocket()
uint32_t sendUDPpacket(uint16_t port_to, uint16_t port_src, uint8_t *data) {
    struct udp_pocket * udp_packet = (struct udp_pocket *)
            (data - sizeof(struct udp_header));
    struct ip_header* ip_packet = (struct ip_header*)
            ((char*)udp_packet - sizeof(struct ip_header));

    udp_packet->hader.dst_port = port_to;
    udp_packet->hader.src_port = port_src;

    return microip_send_ip_packet(ip_packet,
            udp_packet->hader.length, IP_DEFAULT_TTL, IPPROTO_UDP);
}

