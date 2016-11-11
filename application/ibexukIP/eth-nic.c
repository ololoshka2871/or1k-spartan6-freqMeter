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

#include "minimac2.h"

BYTE nic_is_linked;
WORD nic_tx_len;
BYTE nic_rx_packet_waiting_to_be_dumped;

extern void init_eth_timers();

//------------------------------------------------------------------------------

static enum enMiniMACRxSlots rxWorkingSlot;

static BYTE * rxPointer;

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

    return miniMAC_rxCount(rxWorkingSlot);
}



//**********************************************************
//**********************************************************
//********** CHECK IF OK TO START A NEW TX PACKET **********
//**********************************************************
//**********************************************************
BYTE nic_ok_to_do_tx (void) {
    return miniMAC_txRemaning() == 0;
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
void nic_move_pointer (WORD move_pointer_to_ethernet_byte) {
    rxPointer = miniMAC_rxSlotData(rxWorkingSlot) + move_pointer_to_ethernet_byte;
}



//****************************************
//****************************************
//********** NIC DUMP RX PACKET **********
//****************************************
//****************************************
//Discard any remaining bytes in the current RX packet and free up the nic for the next rx packet
void nic_rx_dump_packet (void) {
    miniMAC_resetRxSlot(rxWorkingSlot);
    rxWorkingSlot = MINIMAC_RX_SLOT_INVALID;
}



//**********************************
//**********************************
//********** NIC SETUP TX **********
//**********************************
//**********************************
//Checks the nic to see if it is ready to accept a new tx packet.  If so it sets up the nic ready for the first byte of the data area to be sent.
//Returns 1 if nic ready, 0 if not.
BYTE nic_setup_tx (void) {

}




//********************************************
//********************************************
//********** NIC TX WRITE NEXT BYTE **********
//********************************************
//********************************************
//(nic_setup_tx must have already been called)
//The nic stores the ethernet tx in words.  This routine deals with this and allows us to work in bytes.
void nic_write_next_byte (BYTE data) {

}



//*************************************
//*************************************
//********** NIC WRITE ARRAY **********
//*************************************
//*************************************
//(nic_setup_tx must have already been called)
void nic_write_array (BYTE *array_buffer, WORD array_length) {

}



//*********************************************************
//*********************************************************
//********** NIC WRITE WORD AT SPECIFIC LOCATION **********
//*********************************************************
//*********************************************************
//byte_address must be word aligned
void nic_write_tx_word_at_location (WORD byte_address, WORD data) {

}




//**************************************************
//**************************************************
//********** WRITE ETHERNET HEADER TO NIC **********
//**************************************************
//**************************************************
//nic_setup_tx() must have been called first
void write_eth_header_to_nic (MAC_ADDR *remote_mac_address, WORD ethernet_packet_type) {

}



//**************************************************************
//**************************************************************
//********** TRANSMIT THE PACKET IN THE NIC TX BUFFER **********
//**************************************************************
//**************************************************************
void nix_tx_packet (void) {

}


















