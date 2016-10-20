#ifndef ARP_H
#define ARP_H


#define ARP_PACKET_LENGTH   (sizeof(struct arp_frame) + sizeof(struct ethernet_header))

struct arp_frame {
    unsigned short hwtype;
    unsigned short proto;
    unsigned char hwsize;
    unsigned char protosize;
    unsigned short opcode;
    unsigned char sender_mac[6];
    unsigned int sender_ip;
    unsigned char target_mac[6];
    unsigned int target_ip;
    unsigned char padding[18];
} __attribute__((packed));

void process_arp(const struct arp_frame *rx_arp);

#endif // ARP_H
