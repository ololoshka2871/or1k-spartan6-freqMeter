#ifndef MINMAC_H
#define MINMAC_H

#include "mem_map.h"

#ifndef MAC_TX_MEM_BASE
#warning "MAC_TX_MEM_BASE undefined!"
#define MAC_TX_MEM_BASE         0x11200000
#endif

#ifndef MAC_RX_MEM_BASE
#warning "MAC_RX_MEM_BASE undefined!"
#define MAC_RX_MEM_BASE         0x11300000
#endif

#ifndef MAC_CTL_BASE
#warning "MAC_CTL_BASE undefined!"
#define MAC_CTL_BASE            0x11100000
#endif

#ifndef MTU
#warning "MTU undefined!"
#define MTU                     1530

//------------------------------------------------------------------------------

// Registers

#define MINMAC_RST_CTL          (*(REG32 (MAC_CTL_BASE + 0x0)))
#define MINMAC_RST_RX           (1 << 0)
#define MINMAC_RST_RX           (1 << 1)

#define MINMAC_MDIO_BB          (*(REG32 (MAC_CTL_BASE + 0x4)))
#define MINMAC_MDIO_BB_DO       (1 << 0)
#define MINMAC_MDIO_BB_OE       (1 << 1)
#define MINMAC_MDIO_BB_CLK      (1 << 2)

#define MINMAC_SLOT0_STATE      (*(REG32 (MAC_CTL_BASE + 0x8)))
#define MINMAC_SLOT0_ADDR       (*(REG32 (MAC_CTL_BASE + 0xb)))
#define MINMAC_SLOT1_STATE      (*(REG32 (MAC_CTL_BASE + 0x10)))
#define MINMAC_SLOT1_ADDR       (*(REG32 (MAC_CTL_BASE + 0x14)))
#define MINMAC_SLOT2_STATE      (*(REG32 (MAC_CTL_BASE + 0x18)))
#define MINMAC_SLOT2_ADDR       (*(REG32 (MAC_CTL_BASE + 0x1b)))
#define MINMAC_SLOT3_STATE      (*(REG32 (MAC_CTL_BASE + 0x20)))
#define MINMAC_SLOT4_ADDR       (*(REG32 (MAC_CTL_BASE + 0x24)))
#define MINMAC_TX_ADDR          (*(REG32 (MAC_CTL_BASE + 0x20)))
#define MINMAC_TX_REMAINING     (*(REG32 (MAC_CTL_BASE + 0x24)))

enum enMinmacSlotStates {
    MINMAC_SLOT_STATE_DISABLED = 0b00,
    MINMAC_SLOT_STATE_READY = 0b01,
    MINMAC_SLOT_STATE_DATA_RESSIVED = 0b10,
    MINMAC_SLOT_STATE_INVALID = 0b11,
};

enum enMinmacRxSlots {
    MINMAC_RX_SLOT0 = 0,
    MINMAC_RX_SLOT1 = 1,
    MINMAC_RX_SLOT2 = 2,
    MINMAC_RX_SLOT3 = 3,
};

enum enMinmacErrorCodes {
    MINMAC_OK = 0,
    MINMAC_E_NOMEM = 1,
};

//------------------------------------------------------------------------------

void minmac_control(bool rx_enable, bool tx_enable);

// interrupt handlers
void minmac_rx_isr(unsigned int * registers);
void minmac_tx_isr(unsigned int * registers);

enum enMinmacErrorCodes minmac_rx_static_slot_alocate();


#endif // MINMAC_H
