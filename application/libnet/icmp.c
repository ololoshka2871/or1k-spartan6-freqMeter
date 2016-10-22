/*!     \file network/icmp.c
 *      \brief ICMP (Internet Control Message Protocol) layer.
 *      \author Andrea Righi <drizzt@inwind.it>
 *      \date Last update: 2003-11-09
 *      \note Copyright (&copy;) 2003 Andrea Righi
 */

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "microip.h"
#include "cksum.h"

#include "icmp.h"

void process_icmp(struct icmp_frame *packet, size_t size)
{
    icmp_t *icmp_packet = &packet->icmp;
    if( icmp_packet->icmp_type == ICMP_ECHO) {
        send_icmp_packet(packet->ip.SourceIP,
                         ICMP_ECHOREPLY,
                         (uint8_t *)icmp_packet + sizeof(icmp_t),
                         size-sizeof(icmp_t)
                         );
    }
}

int send_icmp_packet(uint32_t ip_to, uint8_t message, uint8_t *data, size_t len)
{
    icmp_t* icmp_packet;
    struct ip_header* ip_packet =
            microip_allocate_ip_pocket((uint8_t**)&icmp_packet,
                                       ip_to, sizeof(icmp_t) + len);

    if (!ip_packet) {
        return -ENOMEM;
    }

    // Create the ICMP header                                       //
    icmp_packet->icmp_type = message;
    icmp_packet->icmp_code = 0;
    icmp_packet->icmp_chk = 0;

    // Copy the data into the packet                                //
    memcpy((uint8_t*)icmp_packet + sizeof(icmp_t), data, len);
    len += sizeof(icmp_t);

    // Calculate the checksum of the ICMP message                   //
    icmp_packet->icmp_chk = cksum((uint16_t*)icmp_packet, len);

    // Send the IP packet                                           //
    microip_send_ip_packet(ip_packet, len, IP_DEFAULT_TTL, IPPROTO_ICMP);
    return(len);
}

