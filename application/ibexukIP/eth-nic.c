/*
IBEX UK LTD http://www.ibexuk.com
Electronic Product Design Specialists
RELEASED SOFTWARE

The MIT License (MIT)

Copyright (c) 2013, IBEX UK Ltd, http://ibexuk.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//Project Name:		TCP/IP DRIVER

#include <stddef.h>
#include <string.h>

#include "main.h"					//Global data type definitions (see https://github.com/ibexuk/C_Generic_Header_File )
#include "eth-main.h"		//Include before our header file
#define	NIC_C
#include "eth-nic.h"

#ifdef STACK_USE_UDP
#include "eth-udp.h"
#endif

#ifdef STACK_USE_TCP
#include "eth-tcp.h"
#endif

#include "crc32.h"
#include "minimac2.h"

BYTE nic_is_linked;
WORD nic_tx_len;
BYTE nic_rx_packet_waiting_to_be_dumped;

extern void init_eth_timers();

//------------------------------------------------------------------------------

static enum enMiniMACRxSlots rxWorkingSlot;

static BYTE * rxPointer;
static BYTE * txPointer;


//************************************
//************************************
//********** INITIALISE NIC **********
//************************************
//************************************
//Call with:
//0 = allow speed 10 / 100 Mbps
void nic_initialise (BYTE init_config) {
    (void)init_config;

    rxWorkingSlot = MINIMAC_RX_SLOT_INVALID;
    rxPointer = NULL;

    miniMAC_init();

    init_eth_timers();
}



//********************************************
//********************************************
//********** CHECK FOR NIC ACTIVITY **********
//********************************************
//********************************************
//Returns 0 if no rx waiting (other nic activities may have been processed) or the number of bytes in the packet if rx is waiting
WORD nic_check_for_rx (void) {
    miniMAC_resetIfError();

    rxWorkingSlot = miniMAC_findReadySlot();
    if (rxWorkingSlot == MINIMAC_RX_SLOT_INVALID)
        return 0;

    miniMAC_acceptSlot(rxWorkingSlot);
    rxPointer = miniMAC_rxSlotData(rxWorkingSlot);
    uint16_t rxlen = miniMAC_rxCount(rxWorkingSlot);

    if (rxlen < ETHERNET_FRAME_SIZE_MIN) goto __ressive_error;

    // check crc32
    uint32_t ressived_crc = ((uint32_t)rxPointer[rxlen-1] << 24) |
                ((uint32_t)rxPointer[rxlen-2] << 16) |
                ((uint32_t)rxPointer[rxlen-3] <<  8) |
                ((uint32_t)rxPointer[rxlen-4]);
    uint32_t computed_crc =
            crc32(rxPointer, rxlen - sizeof(ressived_crc));

    if (ressived_crc != computed_crc) goto __ressive_error;

    return rxlen;

__ressive_error:
    miniMAC_resetRxSlot(rxWorkingSlot);
    rxWorkingSlot = MINIMAC_RX_SLOT_INVALID;
    return 0;
}



//**********************************************************
//**********************************************************
//********** CHECK IF OK TO START A NEW TX PACKET **********
//**********************************************************
//**********************************************************
BYTE nic_ok_to_do_tx (void) {
    return miniMAC_isReadyToTx();
}


//****************************************
//****************************************
//********** NIC READ NEXT BYTE **********
//****************************************
//****************************************
//(nic_setup_read_data must have already been called)
//The nic stores the ethernet rx in little endian words.  This routine deals with this and allows us to work in bytes.
//Returns 1 if read successful, 0 if there are no more bytes in the rx buffer
BYTE nic_read_next_byte (BYTE *data) {
    *data = *rxPointer;
    ++rxPointer;
    return TRUE;
}


//************************************
//************************************
//********** NIC READ ARRAY **********
//************************************
//************************************
//(nic_setup_read_data must have already been called)
BYTE nic_read_array (BYTE *array_buffer, WORD array_length) {
    memcpy(array_buffer, rxPointer, array_length);
    rxPointer += array_length;
    return TRUE;
}



//**************************************
//**************************************
//********** NIC MOVE POINTER **********
//**************************************
//**************************************
//Moves the pointer to a specified byte ready to be read next, with a value of 0 = the first byte of the Ethernet header
void nic_move_pointer (SIGNED_WORD move_pointer_to_ethernet_byte) {
    rxPointer = miniMAC_rxSlotData(rxWorkingSlot) + move_pointer_to_ethernet_byte;
}



//****************************************
//****************************************
//********** NIC DUMP RX PACKET **********
//****************************************
//****************************************
//Discard any remaining bytes in the current RX packet and free up the nic for the next rx packet
void nic_rx_dump_packet (void) {
    if (rxWorkingSlot != MINIMAC_RX_SLOT_INVALID) {
        miniMAC_resetRxSlot(rxWorkingSlot);
        rxWorkingSlot = MINIMAC_RX_SLOT_INVALID;
    }
}



//**********************************
//**********************************
//********** NIC SETUP TX **********
//**********************************
//**********************************
//Checks the nic to see if it is ready to accept a new tx packet.  If so it sets up the nic ready for the first byte of the data area to be sent.
//Returns 1 if nic ready, 0 if not.
BYTE nic_setup_tx (void) {
    txPointer = miniMAC_txSlotData();
    nic_tx_len = 0;

    return TRUE;
}

//**************************************
//**************************************
//********** NIC MOVE TX POINTER *******
//**************************************
//**************************************
//Moves the pointer to a specified byte ready to be read next, with a value of 0 = the first byte of the Ethernet header
void nic_tx_writen_directly (WORD bytes_writen) {
    txPointer += bytes_writen;
    nic_tx_len += bytes_writen;
}

//********************************************
//********************************************
//********** NIC TX WRITE NEXT BYTE **********
//********************************************
//********************************************
//(nic_setup_tx must have already been called)
//The nic stores the ethernet tx in words.  This routine deals with this and allows us to work in bytes.
void nic_write_next_byte (BYTE data) {
    *txPointer = data;
    ++txPointer;
    ++nic_tx_len;
}



//*************************************
//*************************************
//********** NIC WRITE ARRAY **********
//*************************************
//*************************************
//(nic_setup_tx must have already been called)
BYTE nic_write_array (BYTE *array_buffer, WORD array_length) {
    memcpy(txPointer, array_buffer, array_length);
    txPointer += array_length;
    nic_tx_len += array_length;
    return TRUE;
}

//*************************************
//*************************************
//***** NIC GET POINTER TO WRITE ******
//*************************************
//*************************************
//(nic_setup_tx must have already been called)
BYTE * nic_get_wrpointer() {
    return txPointer;
}


//*********************************************************
//*********************************************************
//********** NIC WRITE WORD AT SPECIFIC LOCATION **********
//*********************************************************
//*********************************************************
//byte_address must be word aligned
void nic_write_tx_word_at_location (WORD byte_address, WORD data) {
    BYTE *wp = miniMAC_txSlotData() + byte_address;
    BYTE *src = (BYTE *)&data;
    *wp++ = *src++;
    *wp   = *src;
}




//**************************************************
//**************************************************
//********** WRITE ETHERNET HEADER TO NIC **********
//**************************************************
//**************************************************
//nic_setup_tx() must have been called first
void write_eth_header_to_nic (MAC_ADDR *remote_mac_address, WORD ethernet_packet_type) {
    struct ethernet_header *hader = (struct ethernet_header *)txPointer;

#ifdef PACKED_STRUCT
    memcpy(hader->destmac, remote_mac_address->v, MAC_ADDR_LENGTH);
    memcpy(hader->srcmac, our_mac_address.v, MAC_ADDR_LENGTH);
#else
    hader->destmac[0] = remote_mac_address->v[0];
    hader->destmac[1] = remote_mac_address->v[1];
    hader->destmac[2] = remote_mac_address->v[2];
    hader->destmac[3] = remote_mac_address->v[3];
    hader->destmac[4] = remote_mac_address->v[4];
    hader->destmac[5] = remote_mac_address->v[5];

    hader->srcmac[0] = our_mac_address.v[0];
    hader->srcmac[1] = our_mac_address.v[1];
    hader->srcmac[2] = our_mac_address.v[2];
    hader->srcmac[3] = our_mac_address.v[3];
    hader->srcmac[4] = our_mac_address.v[4];
    hader->srcmac[5] = our_mac_address.v[5];
#endif

    hader->ethertype = ethernet_packet_type;

    txPointer += sizeof(struct ethernet_header);
    nic_tx_len = sizeof(struct ethernet_header);
}



//**************************************************************
//**************************************************************
//********** TRANSMIT THE PACKET IN THE NIC TX BUFFER **********
//**************************************************************
//**************************************************************
void nic_tx_packet (void) {
    BYTE* pocket_start = miniMAC_txSlotData();

    if (nic_tx_len < ETHERNET_FRAME_SIZE_MIN - sizeof(DWORD)) {
        DWORD filling_butes = ETHERNET_FRAME_SIZE_MIN -
                sizeof(DWORD) - nic_tx_len;
        memset(txPointer, 0x00, filling_butes);

        nic_tx_len = ETHERNET_FRAME_SIZE_MIN - sizeof(DWORD);
        txPointer = pocket_start + nic_tx_len;
    }

    DWORD crc = crc32(pocket_start, nic_tx_len);
    *txPointer++ = crc;
    *txPointer++ = crc >> 8;
    *txPointer++ = crc >> 16;
    *txPointer   = crc >> 24;

    nic_tx_len += sizeof(crc);

    miniMAC_startTransmission(nic_tx_len);
}

