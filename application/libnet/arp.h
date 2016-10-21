#ifndef ARP_H
#define ARP_H

#include <stdint.h>

#define ARP_PACKET_LENGTH   (sizeof(struct arp_frame) + sizeof(struct ethernet_header))

struct arp_frame {
    unsigned short hwtype;
    unsigned short proto;
    unsigned char hwsize;
    unsigned char protosize;
    unsigned short opcode;
    unsigned char sender_mac[MAC_ADDR_SIZE];
    unsigned int sender_ip;
    unsigned char target_mac[MAC_ADDR_SIZE];
    unsigned int target_ip;
    unsigned char padding[18];
} __attribute__((packed));

void process_arp(const struct arp_frame *rx_arp);

void arp_init(void);
uint8_t *arp_ip2MAC(uint32_t ip);
void arp_add_entry(uint8_t* MAC, uint32_t ip);

#endif // ARP_H
