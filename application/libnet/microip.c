/*
 * This file is part of m-labs/misoc project
 * https://github.com/m-labs/misoc
 * Modified by Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 */

#include <string.h>
#include <assert.h>

#include "minimac.h"
#include "cache.h"
#include "arp.h"
#include "udp.h"
#include "icmp.h"
#include "cksum.h"

#include "microip.h"

#define IHL_NO_OPTS     (sizeof(struct ip_header) / sizeof(uint32_t))

struct ethernet_frame {
    struct ethernet_header eth_header;
    union {
        struct arp_frame arp;
        struct udp_frame udp;
        struct icmp_frame icmp;
        struct ip_header ip_hader;
    } contents;
} __attribute__((packed));

typedef union {
    struct ethernet_frame frame;
    unsigned char raw[1532];
} ethernet_buffer;

//------------------------------------------------------------------------------

unsigned int my_ip;

//------------------------------------------------------------------------------

uint32_t ntohl(uint32_t net32) {
    return net32;
}

uint16_t ntohs(uint16_t net16) {
    return net16;
}

//------------------------------------------------------------------------------

static void process_frame(ethernet_buffer * rxbuffer, uint16_t rxlen) {
    cache_dflush();

    if((rxbuffer->frame.eth_header.ethertype == ETHERTYPE_ARP) &&
            (rxlen == ARP_PACKET_LENGTH)) {
        process_arp(&rxbuffer->frame.contents.arp);
        return;
    }

    if(rxbuffer->frame.eth_header.ethertype == ETHERTYPE_IP) {
        struct ip_header* ip = &rxbuffer->frame.contents.ip_hader;

        if(ip->Version != IP_IPV4) return;
        if(ip->FragmentOffset != IP_DONT_FRAGMENT) return;
        if(ip->DestinationIP != my_ip) return;
        if(ip->IHL > IHL_NO_OPTS) return; // drop options
        if(cksum((uint16_t*)ip, ip->IHL * sizeof(uint32_t))) return; // mast be 0

        // add to sender to arp table
        arp_add_entry(rxbuffer->frame.eth_header.srcmac, ip->SourceIP);

        switch (ip->Protocol) {
        case IPPROTO_ICMP:
            if (rxlen >= ICMP_PACKET_LENGTH)
                process_icmp(&rxbuffer->frame.contents.icmp,
                             rxlen - sizeof(struct ip_header)
                             - sizeof(struct ethernet_header));
            return;
        case IPPROTO_UDP:
            if (rxlen >= UDP_PACKET_LENGTH)
                process_udp(&rxbuffer->frame.contents.udp);
            return;
        }
    }
}

void microip_service(void) {
    enum enMiniMACRxSlots slot;
    enum enMiniMACErrorCodes err;

    ethernet_buffer* pyload;
    uint16_t size;

    err = miniMAC_getpointerRxDatarRxData(
                &slot, (union uethernet_buffer**)&pyload, &size);
    if (err == MINIMAC_OK) {
        process_frame(pyload, size);
        miniMAC_reset_rx_slot(slot);
    }
}


void microip_start(unsigned int ip) {
    my_ip = ip;
    arp_init();
    miniMAC_control(true, true);
}

struct ip_header *
microip_allocate_ip_pocket(uint8_t **ppyload, uint32_t destIP, size_t ip_pyload_size) {
    assert(ppyload);
    assert(destIP);

    ip_pyload_size += sizeof(struct ip_header); // total_size

    uint8_t* tx_slot = miniMAC_tx_slot_allocate(ip_pyload_size);
    if (!tx_slot) {
        *ppyload = NULL;
        return NULL;
    }

    uint8_t * MAC = arp_ip2MAC(destIP);
    if (!MAC)  {
        *ppyload = NULL;
        return NULL;
    }

    struct ip_header *packet_hader = miniMAC_slot_prepare(MAC, ETHERTYPE_IP, tx_slot);
    memset(packet_hader, 0, sizeof(struct ip_header));
    // set default values
    packet_hader->DestinationIP = destIP;
    packet_hader->Version = IP_IPV4;
    packet_hader->SourceIP = my_ip;
    packet_hader->TypeOfService = 0;
    packet_hader->FragmentOffset = IP_DONT_FRAGMENT;
    packet_hader->Identification = 0;
    packet_hader->IHL = IHL_NO_OPTS;
    packet_hader->TotalLength = ip_pyload_size;

    *ppyload = ((uint8_t*)packet_hader) + sizeof(struct ip_header);

    return packet_hader;
}

uint32_t microip_send_ip_packet(struct ip_header *packet,
                      size_t paylod_size, uint8_t TTL, uint8_t protocol)
{
    packet->Protocol = protocol;
    packet->TTL = TTL;
    packet->HaderCKSumm = cksum(packet, packet->IHL * sizeof(uint32_t));

    uint8_t* tx_slot = ((uint8_t*)packet) - sizeof(struct ethernet_header);

    return miniMAC_slot_complite_and_send(tx_slot);
}
