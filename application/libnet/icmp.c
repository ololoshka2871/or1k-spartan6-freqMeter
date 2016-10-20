/*!     \file network/icmp.c
 *      \brief ICMP (Internet Control Message Protocol) layer.
 *      \author Andrea Righi <drizzt@inwind.it>
 *      \date Last update: 2003-11-09
 *      \note Copyright (&copy;) 2003 Andrea Righi
 */

#include <stdint.h>

#include "icmp.h"


#if 0
//! \brief Process an ICMP packet received from the IP layer.
//! \param packet The ICMP packet received.
void to_icmp_layer(ip_t *packet)
{
        int size = ntohs(packet->ip_len) - (packet->ip_hdr_len)*sizeof(uint32_t);
        icmp_t *icmp_packet = (icmp_t *)((void *)packet + (packet->ip_hdr_len)*sizeof(uint32_t));
#if 0
        kprintf(        "\n\ricmp_type:%02X, icmp_code:%02X, icmp_chk:%04X"
                "\n\rping_id:%04X, ping_seq:%u"
                "\n\rping %u bytes",
                icmp_packet->icmp_type,
                icmp_packet->icmp_code,
                icmp_packet->icmp_chk,
                ((icmp_ping_t *)icmp_packet)->ping_id,
                ((icmp_ping_t *)icmp_packet)->ping_seq,
                size-sizeof(icmp_ping_t)
        );
        kprintf( "\n\rlocal checksum = %04X", ip_checksum(icmp_packet, size) );
#endif
        // Calculate the header checksum                                //
        if ( ip_checksum(icmp_packet, size) )
        {
                kprintf("\n\ricmp: header checksum error!!!");
                return;
        }
        // Identify the message type                                    //
        switch( icmp_packet->icmp_type )
        {
                case ICMP_ECHO:
                        // Print the echo request to stdout             //
                        #ifdef ICMP_DEBUG
                        kprintf(        "\n\recho request %u(%u) bytes "
                                "%u.%u.%u.%u > %u.%u.%u.%u id=%u seq=%u ttl=%u",
                                size,
                                size - sizeof(icmp_ping_t),

                                IP_A(ntohl(packet->ip_src)), IP_B(ntohl(packet->ip_src)),
                                IP_C(ntohl(packet->ip_src)), IP_D(ntohl(packet->ip_src)),

                                IP_A(ntohl(packet->ip_dst)), IP_B(ntohl(packet->ip_dst)),
                                IP_C(ntohl(packet->ip_dst)), IP_D(ntohl(packet->ip_dst)),

                                ((icmp_ping_t *)icmp_packet)->ping_id,
                                ((icmp_ping_t *)icmp_packet)->ping_seq,
                                packet->ip_ttl
                        );
                        #endif
                        // Check if we must send the echo reply message //
                        if (
                                (packet->ip_dst==get_host_ip()) ||
                                (packet->ip_dst==get_host_bcast()) ||
                                (packet->ip_dst==INADDR_BROADCAST)
                        )
                        {
                                #ifdef ICMP_DEBUG
                                // Notify that we're sending the reply  //
                                kprintf(        "\n\recho reply %u(%u) bytes "
                                        "%u.%u.%u.%u > %u.%u.%u.%u id=%u seq=%u ttl=%u",
                                        size,
                                        size - sizeof(icmp_ping_t),

                                        IP_A(ntohl(get_host_ip())), IP_B(ntohl(get_host_ip())),
                                        IP_C(ntohl(get_host_ip())), IP_D(ntohl(get_host_ip())),

                                        IP_A(ntohl(packet->ip_src)), IP_B(ntohl(packet->ip_src)),
                                        IP_C(ntohl(packet->ip_src)), IP_D(ntohl(packet->ip_src)),

                                        ((icmp_ping_t *)icmp_packet)->ping_id,
                                        ((icmp_ping_t *)icmp_packet)->ping_seq,
                                        IP_DEFAULT_TTL
                                );
                                #endif
                                // Send the echo reply                  //
                                send_icmp_packet(
                                        packet->ip_src,
                                        ICMP_ECHOREPLY,
                                        (void *)icmp_packet + sizeof(icmp_t),
                                        size-sizeof(icmp_t)
                                );
                        }
                break;

                case ICMP_ECHOREPLY:
                        // Show the echo reply message                  //
                        kprintf("\n\recho reply %u.%u.%u.%u > %u.%u.%u.%u id=%u seq=%u ttl=%u",

                                IP_A(ntohl(packet->ip_src)), IP_B(ntohl(packet->ip_src)),
                                IP_C(ntohl(packet->ip_src)), IP_D(ntohl(packet->ip_src)),

                                IP_A(ntohl(packet->ip_dst)), IP_B(ntohl(packet->ip_dst)),
                                IP_C(ntohl(packet->ip_dst)), IP_D(ntohl(packet->ip_dst)),

                                ((icmp_ping_t *)icmp_packet)->ping_id,
                                ((icmp_ping_t *)icmp_packet)->ping_seq,
                                packet->ip_ttl
                        );
                        // TODO:                                        //
                        // Insert the message into the icmp recv buffer //
                break;

                default:
                        kprintf("\n\ricmp: ??? %u.%u.%u.%u > %u.%u.%u.%u id=%u seq=%u ttl=%u",

                                IP_A(ntohl(packet->ip_src)), IP_B(ntohl(packet->ip_src)),
                                IP_C(ntohl(packet->ip_src)), IP_D(ntohl(packet->ip_src)),

                                IP_A(ntohl(packet->ip_dst)), IP_B(ntohl(packet->ip_dst)),
                                IP_C(ntohl(packet->ip_dst)), IP_D(ntohl(packet->ip_dst)),

                                ((icmp_ping_t *)icmp_packet)->ping_id,
                                ((icmp_ping_t *)icmp_packet)->ping_seq,
                                packet->ip_ttl
                        );
                break;
        }
}

//! \brief Send an ICMP packet.
//! \param ip_to The IP destination address (in network format).
//! \param message The ICMP message to send.
//! \param data The data buffer to send into the ICMP packet.
//! \param len The size of the data buffer.
//! \return
//!     \li The number of bytes sent in case of success;
//!     \li a negative value if an error occurs.
int send_icmp_packet(in_addr_t ip_to, uint8_t message, uint8_t *data, size_t len)
{
        icmp_t *packet;
        int tot_len;

        packet = kmalloc(len + sizeof(icmp_t));
        if (packet == NULL)
                // Out of memory!                                       //
                return(-ENOMEM);

        // Create the ICMP header                                       //
        packet->icmp_type = message;
        packet->icmp_code = 0;
        packet->icmp_chk = 0;

        // Copy the data into the packet                                //
        memcpy(packet + 1, data, len);
        len += sizeof(icmp_t);

        // Calculate the checksum of the ICMP message                   //
        packet->icmp_chk = ip_checksum(packet, len);

        // Send the IP packet                                           //
        tot_len = send_ip_packet(ip_to, packet, len, IP_DEFAULT_TTL, IPPROTO_ICMP);

        #ifdef DEBUG
        kprintf("\n\r%u bytes sent from ip layer", tot_len);
        #endif

        // Free the memory of the packet                                //
        kfree(packet);

        if ( tot_len < 0 )
                // Something wrong from at the IP layer                 //
                return(tot_len);

        return(len);
}

#endif
