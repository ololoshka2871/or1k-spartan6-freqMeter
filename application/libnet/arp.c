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

struct arp_table_entry {
    uint8_t MAC[MAC_ADDR_SIZE];
    uint8_t uses;
    uint32_t IP;
};

#ifndef ARP_TABLE_SIZE
#warning "ARP_TABLE_SIZE not defined, assume 3"
#define ARP_TABLE_SIZE 3
#endif

static struct arp_table_entry arp_table[ARP_TABLE_SIZE];

#define ARP_OPCODE_REQUEST  0x0001
#define ARP_OPCODE_REPLY    0x0002

#define ARP_HWTYPE_ETHERNET 0x0001
#define ARP_PROTO_IP        0x0800

void process_arp(const struct arp_frame *rx_arp) {
    if(rx_arp->hwtype != ARP_HWTYPE_ETHERNET) return;
    if(rx_arp->proto != ARP_PROTO_IP) return;
    if(rx_arp->hwsize != 6) return;
    if(rx_arp->protosize != 4) return;
    if (rx_arp->opcode == ARP_OPCODE_REPLY) {
        arp_add_entry((uint8_t*)rx_arp->sender_mac, rx_arp->sender_ip);
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

uint8_t *arp_ip2MAC(uint32_t ip) {
    for(uint32_t i = 0; i < ARP_TABLE_SIZE; ++i) {
        struct arp_table_entry * entry = &arp_table[i];
        if (entry->IP == ip) {
            entry->uses = 0xff;
            return entry->MAC;
        } else {
            if (entry->uses)
                entry->uses--;
        }
    }

    // TODO: send arp request to resolv mac

    return NULL;
}

static void arp_set_table_enty(struct arp_table_entry * entry,
                          uint8_t *MAC, uint32_t ip) {
    entry->IP = ip;
    memcpy(entry->MAC, MAC, MAC_ADDR_SIZE);
    entry->uses = 0xff;
}

void arp_add_entry(uint8_t *MAC, uint32_t ip) {
    struct arp_table_entry * entry;
    uint8_t uses_min = 0xff;
    uint32_t warst_entry = 0;
    for(uint32_t i = 0; i < ARP_TABLE_SIZE; ++i) {
        entry = &arp_table[i];
        if ((entry->IP == ip) ||
            (memcmp(entry->MAC, MAC, MAC_ADDR_SIZE) == 0)) {
            arp_set_table_enty(entry, MAC, ip);
            return;
        }
        if (entry->uses <= uses_min)
            warst_entry = i;
    }
    arp_set_table_enty(&arp_table[warst_entry], MAC, ip);
}

void arp_init() {
    memset(arp_table, 0, sizeof(arp_table));
}
