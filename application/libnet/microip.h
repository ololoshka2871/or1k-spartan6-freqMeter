#ifndef __MICROUDP_H
#define __MICROUDP_H

#include <stdint.h>

#define IPTOINT(a, b, c, d) ((a << 24)|(b << 16)|(c << 8)|d)

#define IP_IPV4             0x4
#define IP_DONT_FRAGMENT	0x4000
#define IP_TTL              64

// http://stackoverflow.com/questions/30158919/dissecting-a-binary-file-in-c

struct ip_header {
    unsigned Version:4;
    unsigned IHL:4;
    uint8_t TypeOfService;
    uint16_t TotalLength;
    uint16_t Identification;
    uint16_t FragmentOffset;
    uint8_t TTL;
    uint8_t Protocol;
    uint16_t HaderCKSumm;
    uint32_t SourceIP;
    uint32_t DestinationIP;
    // options size = IHL * sizeof(uint32_t) - 20
} __attribute__((packed));

typedef void (*udp_callback)(unsigned int src_ip, unsigned short src_port, unsigned short dst_port, void *data, unsigned int length);

void microudp_start(unsigned int ip);
int microudp_arp_resolve(unsigned int ip);
void *microudp_get_tx_buffer(void);
int microudp_send(unsigned short src_port, unsigned short dst_port, unsigned int length);
void microudp_set_callback(udp_callback callback);
void microudp_service(void);

void eth_init(void);
void eth_mode(void);

uint32_t ntohl(uint32_t net32);
#define htonl(x)    ntohl(x)
uint16_t ntohs(uint16_t net16);
#define htons(x)    ntohs(x)

extern unsigned int my_ip;

#endif /* __MICROUDP_H */
