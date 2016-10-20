/****************************************************************************
 *
 *   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
 *   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include "minimac.h"

#ifndef SIM
#include "irq.h"
#include "crc32.h"
#include "heap.h"
#endif

#ifndef MAC_CTL_BASE
#warning "MAC_CTL_BASE undefined!"
#define MAC_CTL_BASE            (FIO_BASE + 0x00100000)
#endif

#ifndef MAC_TX_MEM_BASE
#warning "MAC_TX_MEM_BASE undefined!"
#define MAC_TX_MEM_BASE         (FIO_BASE + 0x00200000)
#endif

#ifndef MAC_RX_MEM_BASE
#warning "MAC_RX_MEM_BASE undefined!"
#define MAC_RX_MEM_BASE         (FIO_BASE + 0x00300000)
#endif

#ifndef MTU
#warning "MTU undefined!"
#define MTU                     1530
#endif


// Registers
#define REG_OFFSET(N)           (sizeof(uint32_t) * (N))

#define MINIMAC_RST_CTL         (*(REG32 (MAC_CTL_BASE + REG_OFFSET(0))))
#define MINIMAC_RST_RX          (1 << 0)
#define MINIMAC_RST_TX          (1 << 1)

#define MINIMAC_MDIO_BB         (*(REG32 (MAC_CTL_BASE + REG_OFFSET(1))))
#define MINIMAC_MDIO_BB_DO      (1 << 0)
#define MINIMAC_MDIO_BB_DO_0    0
#define MINIMAC_MDIO_BB_DI      (1 << 1) //RO
#define MINIMAC_MDIO_BB_DI_0    0
#define MINIMAC_MDIO_BB_OE      (1 << 2)
#define MINIMAC_MDIO_BB_OE_0    0
#define MINIMAC_MDIO_BB_CLK     (1 << 3)
#define MINIMAC_MDIO_BB_CLK_0   0

#define MINIMAC_SLOT0_STATE     (*(REG32 (MAC_CTL_BASE + REG_OFFSET(2))))
#define MINIMAC_SLOT0_ADDR      (*(REG32 (MAC_CTL_BASE + REG_OFFSET(3))))
#define MINIMAC_SLOT0_COUNT     (*(REG32 (MAC_CTL_BASE + REG_OFFSET(4)))) // RO
#if 1
#define MINIMAC_SLOT_STATE(slot) (*(REG32 (MAC_CTL_BASE + REG_OFFSET(2 + (slot) * 3))))
#define MINIMAC_SLOT_ADDR(slot)  (*(REG32 (MAC_CTL_BASE + REG_OFFSET(3 + (slot) * 3))))
#define MINIMAC_SLOT_COUNT(slot) (*(REG32 (MAC_CTL_BASE + REG_OFFSET(4 + (slot) * 3))))// RO
#else
#define MINIMAC_SLOT_STATE(slot) (*(REG32 (MAC_CTL_BASE + REG_OFFSET(2 + (3 - (slot)) * 3))))
#define MINIMAC_SLOT_ADDR(slot)  (*(REG32 (MAC_CTL_BASE + REG_OFFSET(3 + (3 - (slot)) * 3))))
#define MINIMAC_SLOT_COUNT(slot) (*(REG32 (MAC_CTL_BASE + REG_OFFSET(4 + (3 - (slot)) * 3))))// RO
#endif

#define MINIMAC_TX_SLOT_ADDR    (*(REG32 (MAC_CTL_BASE + REG_OFFSET(14))))
#define MINIMAC_TX_REMAINING    (*(REG32 (MAC_CTL_BASE + REG_OFFSET(15))))

#ifndef ETHERNET_MAC // TODO: Make generation
#warning "Ethernet MAC adress not defined, assuming default 0x0A,0xFF,0xFE,0xFD,0xFC,0xFB"
#define ETHERNET_MAC    0x0A,0xFF,0xFE,0xFD,0xFC,0xFB
#endif

#define MINIMAC_ENABLE_RX(enable) \
    do {\
        if (enable) \
            MINIMAC_RST_CTL &= ~MINIMAC_RST_RX;\
        else \
            MINIMAC_RST_CTL |= MINIMAC_RST_RX;\
    } while (0)

#define MINIMAC_ENABLE_TX(enable) \
    do {\
        if (enable) \
            MINIMAC_RST_CTL &= ~MINIMAC_RST_TX;\
        else \
            MINIMAC_RST_CTL |= MINIMAC_RST_TX;\
    } while (0)

struct stx_slot_alloc_unit {
    struct tx_slot_queue_item {
        uint8_t* next_data;
        uint32_t this_pocket_size;
    } queue_info;
    uint8_t data[MTU];
};

static struct sminiMAC_Stat minimacstat;

uint8_t myMAC[6] = {ETHERNET_MAC};
const uint8_t broadcastMAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static bool rx_quick_verify(enum enMiniMACRxSlots slot) {
    struct ethernet_header *pocket_data = (struct ethernet_header *)MINIMAC_SLOT_ADDR(slot);
    uint32_t ressived_size = MINIMAC_SLOT_COUNT(slot);
    if (memcmp(pocket_data->destmac, myMAC, sizeof(myMAC)))
        if (memcmp(pocket_data->destmac, broadcastMAC, sizeof(myMAC)))
            return false;

    if (pocket_data->ethertype != ETHERTYPE_ARP)
        if (pocket_data->ethertype != ETHERTYPE_IP)
            return false;

    if (ressived_size >= ETHERNET_FRAME_SIZE_MIN)
        if (ressived_size <= MTU)
            return true;

    return false;
}

// interrupt handlers
static void miniMAC_rx_isr(unsigned int * registers) {
    (void)registers;
    enum enMiniMACSlotStates slot_state;
    enum enMiniMACRxSlots slot;
    if (MINIMAC_RST_CTL & MINIMAC_RST_RX) {
        // ressive error
        // слот, который сейчас активен вызвал ошибку приёма, сбросим его
        // затем разрешим работу приёмника
        for (slot = MINIMAC_RX_SLOT0; slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
            uint32_t count = MINIMAC_SLOT_COUNT(slot);
            slot_state = MINIMAC_SLOT_STATE(slot);
            if (count && (slot_state == MINIMAC_SLOT_STATE_READY)) {
                miniMAC_reset_rx_slot(slot);
                minimacstat.pocket_rx_errors++;
            }
        }
        MINIMAC_ENABLE_RX(true);
    }

    for (slot = MINIMAC_RX_SLOT0; slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
        // check slot events
        slot_state = MINIMAC_SLOT_STATE(slot);
        switch (slot_state) {
        case MINIMAC_SLOT_STATE_DISABLED:
        case MINIMAC_SLOT_STATE_READY:
            // no action
            break;
        case MINIMAC_SLOT_STATE_DATA_RESSIVED:
            // valid data ressived, quick verify
            if (rx_quick_verify(slot)) {
                MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_DISABLED;
                continue;
            }
            // drop down
        default:
            miniMAC_reset_rx_slot(slot);
            minimacstat.pocket_rx_errors++;
            break;
        }
    }
}


static void miniMAC_tx_isr(unsigned int * registers) {
    (void)registers;
    struct stx_slot_alloc_unit* sent_mem_block = (struct stx_slot_alloc_unit*)(
        MINIMAC_TX_SLOT_ADDR - sizeof(struct tx_slot_queue_item));
    uint8_t* next_data = sent_mem_block->queue_info.next_data;

    if (next_data) {
        struct stx_slot_alloc_unit* next_alloc_unit = (struct stx_slot_alloc_unit*)(
                next_data - sizeof(struct tx_slot_queue_item));
        MINIMAC_TX_SLOT_ADDR = (uint32_t)next_data;
        MINIMAC_TX_REMAINING = next_alloc_unit->queue_info.this_pocket_size;
    }
#ifdef VERBOSE_DEBUG
    memset(sent_mem_block, 0, sizeof(struct tx_slot_queue_item));
#endif
    free_mac_tx(sent_mem_block);
}

static uint32_t align32(uint32_t x) {
    if (x & 0b11)
        return (x & (~0b11)) + 0b100;
    return x;
}

enum enMiniMACRxSlots miniMAC_findSlotWithState(enum enMiniMACSlotStates state) {
    for (enum enMiniMACRxSlots slot = MINIMAC_RX_SLOT0; slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
        enum enMiniMACSlotStates slot_state = MINIMAC_SLOT_STATE(slot);
        if (slot_state == state)
            return slot;
    }
    return MINIMAC_RX_SLOT_INVALID;
}

void miniMAC_control(bool rx_enable, bool tx_enable) {
#ifndef SIM
    irq_acknowledge(IS_MINIMAC_RX);
    irq_acknowledge(IS_MINIMAC_TX);
#else
    IRQ_STATUS = (1 << IRQ_MINIMAC_RX) | (1 << IRQ_MINIMAC_TX);
#endif

    MINIMAC_ENABLE_RX(rx_enable);
    MINIMAC_ENABLE_TX(tx_enable);

    if (rx_enable)
        for (uint8_t i = 0; i < 4; ++i)
            miniMAC_rx_static_slot_allocate();

#ifdef VERBOSE_DEBUG
    memset(MINIMAC_SLOT_ADDR(0), 0x77, 66);
#endif

#ifndef SIM
    if (rx_enable) {
        set_irq_handler(IS_MINIMAC_RX, miniMAC_rx_isr);
        irq_acknowledge(IS_MINIMAC_RX);
        irq_enable(IS_MINIMAC_RX);
    } else {
        irq_disable(IS_MINIMAC_RX);
    }

    if (tx_enable) {
        set_irq_handler(IS_MINIMAC_TX, miniMAC_tx_isr);
        irq_acknowledge(IS_MINIMAC_TX);
        irq_enable(IS_MINIMAC_TX);
    } else {
        irq_disable(IS_MINIMAC_TX);
    }
#endif
}


enum enMiniMACRxSlots miniMAC_rx_static_slot_allocate() {
    // find unused slot
    enum enMiniMACRxSlots slot =
            miniMAC_findSlotWithState(MINIMAC_SLOT_STATE_DISABLED);
    if (slot == MINIMAC_RX_SLOT_INVALID)
        return slot;

    miniMAC_reset_rx_slot(slot);
    return slot;
}

void miniMAC_reset_rx_slot(enum enMiniMACRxSlots slot) {
    assert(slot < MINIMAC_RX_SLOT_COUNT);
    MINIMAC_SLOT_ADDR(slot) = align32(MAC_RX_MEM_BASE + MTU * (int)slot);
    MINIMAC_SLOT_STATE(slot) = MINIMAC_SLOT_STATE_READY;
}

enum enMiniMACErrorCodes miniMAC_getpointerRxDatarRxData(
        enum enMiniMACRxSlots *pslot, union uethernet_buffer** ppayload,
        uint16_t *ppl_size) {
    enum enMiniMACRxSlots  slot = miniMAC_is_data_ressived();
    enum enMiniMACErrorCodes ret = MINIMAC_OK;
    if (slot < MINIMAC_RX_SLOT_COUNT) {
        if (pslot) *pslot = slot;
        // check slot crc
        uint32_t rxlen = MINIMAC_SLOT_COUNT(slot);
        union uethernet_buffer* rxbuffer = (union uethernet_buffer*)MINIMAC_SLOT_ADDR(slot);
        uint32_t computed_crc;
        uint32_t ressived_crc = ((uint32_t)rxbuffer->raw[rxlen-1] << 24) |
                    ((uint32_t)rxbuffer->raw[rxlen-2] << 16) |
                    ((uint32_t)rxbuffer->raw[rxlen-3] <<  8) |
                    ((uint32_t)rxbuffer->raw[rxlen-4]);
#ifdef SIM
        computed_crc = ressived_crc;
#else
        computed_crc = crc32(rxbuffer->raw, rxlen - sizeof(ressived_crc), 0);
#endif
        if (ressived_crc == computed_crc) {
            if (ppayload && ppl_size) {
                *ppl_size = rxlen - sizeof(ressived_crc);
                *ppayload = rxbuffer;
            }
            minimacstat.pocket_rx++;
            return ret;
        } else {
            ret = MINIMAC_CRC_ERROR;
            minimacstat.pocket_rx_errors++;
        }
        miniMAC_reset_rx_slot(slot);
    } else {
        if (pslot) *pslot = MINIMAC_RX_SLOT_INVALID;
        ret = MINIMAC_NO_DATA_AVALABLE;
    }

    if (ppl_size)   *ppl_size = 0;
    return ret;
}


enum enMiniMACRxSlots miniMAC_is_data_ressived() {
    for (enum enMiniMACRxSlots slot = MINIMAC_RX_SLOT0; slot < MINIMAC_RX_SLOT_COUNT; ++slot) {
        enum enMiniMACSlotStates state = MINIMAC_SLOT_STATE(slot);
        uint32_t ressived_size = MINIMAC_SLOT_COUNT(slot);
        if (ressived_size && (state == MINIMAC_SLOT_STATE_DISABLED))
            return slot;
    }
    return MINIMAC_RX_SLOT_INVALID;
}

void minMAC_stat(struct sminiMAC_Stat* pstst) {
    if (pstst)
        memcpy(pstst, &minimacstat, sizeof(struct sminiMAC_Stat));
}

uint8_t* miniMAC_tx_slot_allocate(size_t pyload_size) {
    size_t alloc_size;
    size_t pocket_size;
    if (pyload_size < 0) {
        alloc_size = get_heap_free_mac_tx();
        pocket_size = alloc_size - sizeof(struct tx_slot_queue_item);
        if (pocket_size < ETHERNET_FRAME_SIZE_MIN)
            return NULL;
        pyload_size = pocket_size - sizeof(struct ethernet_header) - // hader
                sizeof(uint32_t); // crc32
    } else {
        pocket_size = pyload_size + sizeof(struct ethernet_header) + // hader
                sizeof(uint32_t); // crc32
        if (pocket_size < ETHERNET_PAYLOAD_SIZE_MIN) {
            pocket_size = ETHERNET_PAYLOAD_SIZE_MIN;
        }
        alloc_size = pocket_size + sizeof(struct tx_slot_queue_item); // queue data
    }
#ifdef SIM
    if (MINIMAC_TX_REMAINING != 0) {
        return NULL;
    }
    struct stx_slot_alloc_unit *res =
            (struct stx_slot_alloc_unit *)MAC_TX_MEM_BASE;
    res->queue_info.next_data = NULL;
    res->queue_info.this_pocket_size = pyload_size;
    return &res->data;
#else
    struct stx_slot_alloc_unit * res = malloc_mac_tx(alloc_size);
    if (res) {
        res->queue_info.next_data = NULL;
        res->queue_info.this_pocket_size = pocket_size;
#ifdef VERBOSE_DEBUG
        memset(&res->data, 0xAA, sizeof(struct ethernet_header)); // header
        memset(&(((uint8_t*)(&res->data))[sizeof(struct ethernet_header)]),
                0x00, pyload_size); // pyload
        memset(&(((uint8_t*)(&res->data))[pocket_size - sizeof(uint32_t)]),
                0xcc, sizeof(uint32_t));
#endif
        return (uint8_t*)&res->data;
    } else
        return (uint8_t*)res;
#endif
}

uint8_t *miniMAC_slot_prepare(const uint8_t *dest_mac,
                              uint16_t ether_type, uint8_t *slot) {
    memcpy(slot, dest_mac, MAC_ADDR_SIZE);
    slot += MAC_ADDR_SIZE;
    memcpy(slot, myMAC, MAC_ADDR_SIZE);
    slot += MAC_ADDR_SIZE;
    memcpy(slot, &ether_type, sizeof(uint16_t));
    return slot + sizeof(uint16_t);
}

void miniMAC_slot_complite_and_send(uint8_t *slot_data) {
   struct stx_slot_alloc_unit* slot_alloc_unit = (struct stx_slot_alloc_unit*)(
            slot_data - sizeof(struct tx_slot_queue_item));
#ifndef SIM
    uint32_t pocket_size = slot_alloc_unit->queue_info.this_pocket_size;
    uint32_t data_size = pocket_size - sizeof(uint32_t);

    uint32_t crc = crc32(slot_data, data_size, 0);
    slot_data[data_size++] = crc & 0xff;
    slot_data[data_size++] = (crc >> 8) & 0xff;
    slot_data[data_size++] = (crc >> 16) & 0xff;
    slot_data[data_size  ] = (crc >> 24) & 0xff;
#if 1
    irq_disable(IS_MINIMAC_TX);
    if (!MINIMAC_TX_REMAINING) {
        // send now
        MINIMAC_TX_SLOT_ADDR = (uint32_t)slot_data;
        MINIMAC_TX_REMAINING = pocket_size;
        slot_alloc_unit->queue_info.next_data = NULL;
    } else {
        // add to queue
        struct stx_slot_alloc_unit* send_queue_pointer = (struct stx_slot_alloc_unit*)(
                MINIMAC_TX_SLOT_ADDR - sizeof(struct tx_slot_queue_item));
        while(send_queue_pointer->queue_info.next_data != NULL) {
            send_queue_pointer = (struct stx_slot_alloc_unit*)(
                    send_queue_pointer->queue_info.next_data -
                        sizeof(struct tx_slot_queue_item));
        }
        send_queue_pointer->queue_info.next_data = slot_data;
    }
    irq_enable(IS_MINIMAC_TX);
#else
    free_mac_tx(slot_alloc_unit);
#endif

#else
    MINIMAC_TX_SLOT_ADDR = (uint32_t)slot_data;
    MINIMAC_TX_REMAINING = slot_alloc_unit->queue_info.this_pocket_size;
#endif
}
