/*
 * This file is part of m-labs/misoc project
 * https://github.com/m-labs/misoc
 * Modified by Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "minimac.h"

#include "microip.h"

#include "udp.h"

static udp_callback rx_callback = NULL;

void process_udp(struct udp_frame* rx_udp)
{
    if(rx_udp->udp.length < sizeof(struct udp_header)) return;
    // ignore checksumm

#if 0
    if(rx_callback)
        rx_callback(rx_udp->ip.SourceIP, rx_udp->udp.src_port,
                    rx_udp->udp.dst_port, rx_udp->payload,
                    rx_udp->udp.length-sizeof(struct udp_header));
#else
    char buff[256];
    struct sminiMAC_Stat stat;
    minMAC_stat(&stat);
    sprintf(buff, "rx: %d\ttx: %d\trx_drop: %d\t pyload_size: %d\n", stat.pocket_rx,
            stat.pocket_tx, stat.pocket_rx_errors, rx_udp->udp.length - sizeof(struct udp_header));

    send_udp_packet(rx_udp->ip.SourceIP, 4998, rx_udp->udp.dst_port,
                     buff, strlen(buff));
#endif
}

void set_rx_callback(udp_callback callback) {
    rx_callback = callback;
}

int
send_udp_packet(uint32_t ip_to, uint16_t port_to,
                 uint16_t port_src, uint8_t *data, size_t len) {
    struct udp_pocket * udp_packet;
    size_t pocket_len = len + sizeof(struct udp_header);
    struct ip_header* ip_packet =
            microip_allocate_ip_pocket((uint8_t**)&udp_packet, ip_to, pocket_len);
    if (!ip_packet) {
        return -ENOMEM;
    }

    udp_packet->hader.checksum = 0; // checksumm not used
    udp_packet->hader.dst_port = port_to;
    udp_packet->hader.length = pocket_len;
    udp_packet->hader.src_port = port_src;

    memcpy(&udp_packet->payload, data, len);

    microip_send_ip_packet(ip_packet, pocket_len, IP_DEFAULT_TTL, IPPROTO_UDP);
    return len;
}
