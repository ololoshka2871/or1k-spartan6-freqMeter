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

#include "irq.h"
#include "mem_managment.h"

#include "minimac2.h"

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

void minmac_init() {

}

