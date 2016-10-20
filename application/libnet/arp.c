/*
 * This file is part of m-labs/misoc project
 * https://github.com/m-labs/misoc
 * Modified by Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 */

#include <stdint.h>
#include <string.h>

#include "minimac.h"
#include "microip.h"

#include "arp.h"

/* ARP cache - one entry only */
static unsigned char cached_mac[6];
static unsigned int cached_ip;

#define ARP_OPCODE_REQUEST  0x0001
#define ARP_OPCODE_REPLY    0x0002

#define ARP_HWTYPE_ETHERNET 0x0001
#define ARP_PROTO_IP        0x0800

void process_arp(const struct arp_frame *rx_arp) {
    if(rx_arp->hwtype != ARP_HWTYPE_ETHERNET) return;
    if(rx_arp->proto != ARP_PROTO_IP) return;
    if(rx_arp->hwsize != 6) return;
    if(rx_arp->protosize != 4) return;
    if ((rx_arp->opcode == ARP_OPCODE_REPLY) &&
        (rx_arp->sender_ip == cached_ip)) {
        memcpy(cached_mac, rx_arp->sender_mac, sizeof(cached_mac));
        return;
    }
    if ((rx_arp->opcode == ARP_OPCODE_REQUEST) && (rx_arp->target_ip == my_ip)) {
        uint8_t* tx_slot = miniMAC_tx_slot_allocate(sizeof(struct arp_frame));
        if (!tx_slot)
            return; // E_NOMEM

        struct arp_frame *tx_arp = (struct arp_frame*)
                miniMAC_slot_prepare(rx_arp->sender_mac, ETHERTYPE_ARP, tx_slot);

        tx_arp->hwtype = ARP_HWTYPE_ETHERNET;
        tx_arp->proto = ARP_PROTO_IP;
        tx_arp->hwsize = 6;
        tx_arp->protosize = 4;
        tx_arp->opcode = ARP_OPCODE_REPLY;
        tx_arp->sender_ip = my_ip;
        memcpy(tx_arp->sender_mac, myMAC, MAC_ADDR_SIZE);
        tx_arp->target_ip = rx_arp->sender_ip;
        memcpy(tx_arp->target_mac, rx_arp->sender_mac, MAC_ADDR_SIZE);
        miniMAC_slot_complite_and_send(tx_slot);
    }
}
