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


#ifndef ETHERNET_PHY_H
#define ETHERNET_PHY_H

#include "mem_map.h"
#include <stdint.h>

#define MDIO_REG_CTL            (*(REG32 (MDIO_BASE + 0x0)))
#define MDIO_REG_CTL_START      (1 << 31)
#define MDIO_REG_CTL_RW         (1 << 30)
#define MDIO_REG_CTL_IE         (1 << 29)
#define MDIO_REG_CTL_IF         (1 << 28) // RO
#define MDIO_REG_CTL_PHY_ADDR_SH        (5)
#define MDIO_REG_CTL_PHY_ADDR_MSK       (0b11111 << MDIO_REG_CTL_PHY_ADDR_SH)
#define MDIO_REG_CTL_PHY_REGADDR_SH     (0)
#define MDIO_REG_CTL_PHY_REGADDR_MSK    (0b11111 << MDIO_REG_CTL_PHY_REGADDR_SH)

#define MDIO_REG_DATA           (*(REG32 (MDIO_BASE + 0x4)))

// PHY basic ctrl registers
#define PHY_BMCR                (0)     // Basic Mode Control Register
#define PHY_BMCR_RESET          (1 << 15)
#define PHY_BMCR_LOOPBACK       (1 << 14)
#define PHY_BMCR_SPEED100MB     (1 << 13)
#define PHY_BMCR_AUTONEG_EN     (1 << 12)
#define PHY_BMCR_POWER_DOWN     (1 << 11)
#define PHY_BMCR_ISOLATE        (1 << 10)
#define PHY_BMCR_RESET_AUTONEG  (1 << 9)
#define PHY_BMCR_FULL_DUPLEX    (1 << 8)
#define PHY_BMCR_COLISION_TEST  (1 << 7)

#define PHY_BMSR                (1)     // Basic Mode Status Register
#define PHY_BMSR_100BASET4      (1 << 15)
#define PHY_BMSR_100BASET_FD    (1 << 14)
#define PHY_BMSR_100BASET_HD    (1 << 13)
#define PHY_BMSR_10BASET_FD     (1 << 12)
#define PHY_BMSR_10BASET_HD     (1 << 11)
#define PHY_BMSR_PREAMBLE_SUPR  (1 << 6)
#define PHY_BMSR_AutoNegCompl   (1 << 5)
#define PHY_BMSR_RemoteFault    (1 << 4)
#define PHY_BMSR_AutoNegAbility (1 << 3)
#define PHY_BMSR_LinkOk         (1 << 2)
#define PHY_BMSR_JabberDetected (1 << 1)
#define PHY_BMSR_ExtCapability  (1 << 0)


#define PHY_PHYIDR1             (2)     // PHY Identifier Register #1
#define PHY_PHYIDR2             (3)     // PHY Identifier Register #2
#define PHY_PHYIDR2_OUI_LSB_SH  (10)
#define PHY_PHYIDR2_OUI_LSB_MSK (0b111111 << PHY_PHYIDR2_OUI_LSB_SH)
#define PHY_PHYIDR2_VNDR_MDL_SH (4)
#define PHY_PHYIDR2_VNDR_MDL_MSK (0b111111 << PHY_PHYIDR2_VNDR_MDL_SH)
#define PHY_PHYIDR2_MDL_REV_SH  (0)
#define PHY_PHYIDR2_MDL_REV_MSK (0b1111 << PHY_PHYIDR2_MDL_REV_SH)


#define PHY_ANAR                (4)     // Auto-Negotiation Advertisement Register
#define PHY_ANAR_NP             (1 << 15)
#define PHY_ANAR_RF             (1 << 13)
#define PHY_ANAR_ASM_DIR        (1 << 11)
#define PHY_ANAR_PAUSE          (1 << 10)
#define PHY_ANAR_T4             (1 << 19)
#define PHY_ANAR_TX_FD          (1 << 8)
#define PHY_ANAR_TX             (1 << 7)
#define PHY_ANAR_10_FD          (1 << 6)
#define PHY_ANAR_10             (1 << 5)
#define PHY_ANAR_PROTO_SEL_SH   (0)
#define PHY_ANAR_PROTO_SEL_MSK  (0b11111 << PHY_ANAR_PROTO_SEL_SH)

// base page
#define PHY_ANLPAR              (5)     // Auto-Negotiation Link Partner Ability Register
#define PHY_ANLPAR_NP           (1 << 15)
#define PHY_ANLPAR_ACK          (1 << 14)
#define PHY_ANLPAR_RF           (1 << 13)
#define PHY_ANLPAR_ASM_DIR      (1 << 11)
#define PHY_ANLPAR_PAUSE        (1 << 10)
#define PHY_ANLPAR_T4           (1 << 9)
#define PHY_ANLPAR_TX_FD        (1 << 8)
#define PHY_ANLPAR_TX           (1 << 7)
#define PHY_ANLPAR_10_FD        (1 << 6)
#define PHY_ANLPAR_10           (1 << 5)
#define PHY_ANLPAR_PROTO_SEL_SH     (0)
#define PHY_ANLPAR_PROTO_SEL_MSK    (0b11111 << PHY_ANLPAR_PROTO_SEL_SH)

// next page
#define PHY_ANLPARNP            (5)     // Auto-Negotiation Link Partner Ability Register
#define PHY_ANLPARNP_NP         (1 << 15)
#define PHY_ANLPARNP_ACK        (1 << 14)
#define PHY_ANLPARNP_MP         (1 << 13)
#define PHY_ANLPARNP_ACK2       (1 << 12)
#define PHY_ANLPARNP_TOGGLE     (1 << 11)
#define PHY_ANLPARNP_CODE_SH    (0)
#define PHY_ANLPARNP_CODE_MSK   (0b11111111111 << PHY_ANLPARNP_CODE_SH)

#define PHY_ANER                (6)     // Auto-Negotiation Expansion Register
#define PHY_ANER_PDF            (1 << 4)
#define PHY_ANER_LP_NP_ABLE     (1 << 3)
#define PHY_ANER_NP_ABLE        (1 << 2)
#define PHY_ANER_PAGE_RX        (1 << 1)
#define PHY_ANER_LP_AN_ABLE     (1 << 0)


#define PHY_ANNPTR              (7)     // Auto-Negotiation Next Page TX
#define PHY_ANNPTR_NP           (1 << 15)
#define PHY_ANNPTR_MP           (1 << 13)
#define PHY_ANNPTR_ACK2         (1 << 12)
#define PHY_ANNPTR_TOG_TX       (1 << 11)
#define PHY_ANNPTR_CODE_SH      (1 << 0)
#define PHY_ANNPTR_CODE_MSK     (0b11111111111 << PHY_ANNPTR_CODE_SH)


// extended registers
#define PHY_PHYSTS              (0x10)  // PHY Status Register
#define PHY_PHYSTS_MDIX_MODE    (1 << 14)
#define PHY_PHYSTS_RX_ERR_FLAG  (1 << 13)
#define PHY_PHYSTS_POLARITY     (1 << 12)
#define PHY_PHYSTS_FALSE_CARIER (1 << 11)
#define PHY_PHYSTS_SIG_DETECT   (1 << 10)
#define PHY_PHYSTS_DISC_LOCK    (1 << 9)
#define PHY_PHYSTS_PAGERESSIVED (1 << 8)
#define PHY_PHYSTS_MII_INT      (1 << 7)
#define PHY_PHYSTS_REMOTE_FAULT (1 << 6)
#define PHY_PHYSTS_JABBER_DET   (1 << 5)
#define PHY_PHYSTS_AUTO_NEG_OK  (1 << 4)
#define PHY_PHYSTS_LOOPB_EN     (1 << 3)
#define PHY_PHYSTS_FULL_DUPLEX  (1 << 2)
#define PHY_PHYSTS_100Mb        (1 << 1)
#define PHY_PHYSTS_LINK_OK      (1 << 0)


#define PHY_MICR                (0x11)  // MII Interrupt Control Register
#define PHY_MICR_TINT           (1 << 2)
#define PHY_MICR_INTEN          (1 << 1)
#define PHY_MICR_INTOE          (1 << 0)

#define PHY_MISR                (0x12)  // MII Interrupt Status Register
#define PHY_MISR_ED_INT         (1 << 14)
#define PHY_MISR_LINK_INT       (1 << 13)
#define PHY_MISR_SPD_INT        (1 << 12)
#define PHY_MISR_DUP_INT        (1 << 11)
#define PHY_MISR_ANC_INT        (1 << 10)
#define PHY_MISR_FHF_INT        (1 << 9)
#define PHY_MISR_RHF_INT        (1 << 8)
#define PHY_MISR_ED_INT_EN      (1 << 6)
#define PHY_MISR_LINK_INT_EN    (1 << 5)
#define PHY_MISR_SPD_INT_EN     (1 << 4)
#define PHY_MISR_DUP_INT_EN     (1 << 3)
#define PHY_MISR_ANC_INT_EN     (1 << 2)
#define PHY_MISR_FHF_INT_EN     (1 << 1)
#define PHY_MISR_RHF_INT_EN     (1 << 0)

#define PHY_FCSCR               (0x14)  // False Carrier Sense Counter Register

#define PHY_RECR                (0x15)  // Receive Error Counter Register

#define PHY_PCSR                (0x16)  // 100 Mb/s PCS Configuration and Status Register
#define PHY_PCSR_TQ_EN          (1 << 10)
#define PHY_PCSR_SD_FORCE_PMA   (1 << 9)
#define PHY_PCSR_SD_OPTION      (1 << 8)
#define PHY_PCSR_DESC_TIME_2ms  (1 << 7)
#define PHY_PCSR_FORCE_100_OK   (1 << 5)
#define PHY_PCSR_NRZI_BYPASS    (1 << 2)


#define PHY_RBR                 (0x17)  // RMII and Bypass Register
#define PHY_RBR_RMII_MODE       (1 << 5)
#define PHY_RBR_RMII_REV1_0     (1 << 4)
#define PHY_RBR_RX_OVF_STS      (1 << 3)
#define PHY_RBR_RX_UNF_STS      (1 << 2)
#define PHY_RBR_ELAST_BUF_SH    (0)
#define PHY_RBR_ELAST_BUF_MSK   (0b11 << PHY_RBR_ELAST_BUF_SH)

#define PHY_LEDCR               (0x18)  // LED Direct Control Register
#define PHY_LEDCR_DRV_SPDLED    (1 << 5)
#define PHY_LEDCR_DRV_LNKLED    (1 << 4)
#define PHY_LEDCR_DRV_ACTLED    (1 << 3)
#define PHY_LEDCR_SPDLED        (1 << 2)
#define PHY_LEDCR_LNKLED        (1 << 1)
#define PHY_LEDCR_ACTLED        (1 << 0)

#define PHY_PHYCR               (0x19)  // PHY Control Register
#define PHY_PHYCR_MDIX_EN       (1 << 15)
#define PHY_PHYCR_FORCE_MDIX    (1 << 14)
#define PHY_PHYCR_PAUSE_RX      (1 << 13)
#define PHY_PHYCR_PAUSE_TX      (1 << 12)
#define PHY_PHYCR_BIST_FE       (1 << 11)
#define PHY_PHYCR_PSR_15        (1 << 10)
#define PHY_PHYCR_BIST_STATUS   (1 << 9)
#define PHY_PHYCR_BIST_START    (1 << 8)
#define PHY_PHYCR_BP_STRETCH    (1 << 7)
#define PHY_PHYCR_LED_CNFG_SH   (5)
#define PHY_PHYCR_LED_CNFG_MSK  (0b11 << PHY_PHYCR_LED_CNFG_SH)
#define PHY_PHYCR_PHYADDR       (0)
#define PHY_PHYCR_PHYADDR_MSK   (0b11111 << PHY_PHYCR_PHYADDR)


#define PHY_10BTSCR             (0x1a)  // 10Base-T Status/Control Register
#define PHY_10BTSCR_10BT_SERIAL (1 << 15)
#define PHY_10BTSCR_SQUELCH_SH  (9)
#define PHY_10BTSCR_SQUELCH_MSK (0b111 << PHY_10BTSCR_SQUELCH_SH)
#define PHY_10BTSCR_LB_10_D_IS  (1 << 8)
#define PHY_10BTSCR_LP_DIS      (1 << 7)
#define PHY_10BTSCR_FORCE_LNK10 (1 << 6)
#define PHY_10BTSCR_POLARITY    (1 << 4)
#define PHY_10BTSCR_HEARTBEATDIS (1 << 1)
#define PHY_10BTSCR_JABBER_DIS  (1 << 0)


#define PHY_CDCTRL1             (0x1b)  // CD Test Control Register and BIST Extensions Register
#define PHY_CDCTRL1_BIST_ERROR_COUNT_SH     (8)
#define PHY_CDCTRL1_BIST_ERROR_COUNT_MSK    (0b11111111 << PHY_CDCTRL1_BIST_ERROR_COUNT_SH)
#define PHY_CDCTRL1_BIST_CONT_MODE          (1 << 5)
#define PHY_CDCTRL1_CDPATTEN_10             (1 << 5)
#define PHY_CDCTRL1_10MEG_PATT_GAP          (1 << 5)
#define PHY_CDCTRL1_CDPATTSEL_SH            (0)
#define PHY_CDCTRL1_CDPATTSEL_MSK           (0b11 << PHY_CDCTRL1_CDPATTSEL_SH)


#define PHY_EDCR                (0x1d)  // Energy Detect Control Register
#define PHY_EDCR_ED_EN          (1 << 15)
#define PHY_EDCR_ED_AUTO_UP     (1 << 14)
#define PHY_EDCR_ED_AUTO_DOWN   (1 << 13)
#define PHY_EDCR_ED_MAN         (1 << 12)
#define PHY_EDCR_ED_BURST_DIS   (1 << 11)
#define PHY_EDCR_ED_PWR_STATE   (1 << 10)
#define PHY_EDCR_ED_ERR_MET     (1 << 9)
#define PHY_EDCR_ED_DATA_MET    (1 << 8)
#define PHY_EDCR_ED_ERR_COUNT_SH   (4)
#define PHY_EDCR_ED_ERR_COUNT_MSK  (0b1111 << PHY_EDCR_ED_ERR_COUNT_SH)
#define PHY_EDCR_ED_DATA_COUNT_SH  (0)
#define PHY_EDCR_ED_DATA_COUNT_MSK (0b1111 << PHY_EDCR_ED_DATA_COUNT_SH)

//------------------------------------------------------------------------------

uint16_t MDIO_ReadREG_sync(int8_t phy_addr, const uint8_t reg_addr);
void MDIO_WriteREG(int8_t phy_addr, const uint8_t reg_addr,
                           const uint16_t val);
int8_t MDIO_DetectPHY(uint8_t startAddr);
uint32_t MDIO_getConnectionStatus(int8_t phy_addr) ;

#endif // ETHERNET_PHY_H
