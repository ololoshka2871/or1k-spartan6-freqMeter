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

struct udp_pocket {
    struct udp_header hader;
    uint8_t payload[];
};

struct udp_frame {
    struct ip_header ip;
    struct udp_header udp;
    char payload[];
} __attribute__((packed));

typedef void (*udp_callback)(uint32_t src_ip, uint16_t src_port,
                             uint16_t dst_port, void *data,
                             uint32_t length);

void process_udp(struct udp_frame* rx_udp);
void set_rx_callback(udp_callback callback);
int send_udp_packet(uint32_t ip_to, uint16_t port_to, uint16_t port_src,
                     uint8_t *data, size_t len);

#endif // UDP_H
