#ifndef UDP_H
#define UDP_H

#include "microip.h"

#define UDP_PACKET_LENGTH   (sizeof(struct ethernet_header)+sizeof(struct udp_frame))
#define IPPROTO_UDP         17

struct udp_header {
    unsigned short src_port;
    unsigned short dst_port;
    unsigned short length;
    unsigned short checksum;
} __attribute__((packed));

struct udp_frame {
    struct ip_header ip;
    struct udp_header udp;
    char payload[];
} __attribute__((packed));

void process_udp(struct udp_frame* rx_udp);

#endif // UDP_H
