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
//NXP LPC2365 (BUILT IN NETWORK INTERFACE CONTROLLER) WITH KSZ8001




//##################################
//##################################
//########## USING DRIVER ##########
//##################################
//##################################

//Check this header file for defines to setup and any usage notes
//Configure the IO pins as requried in your applications initialisation.



//#######################
//##### MAC ADDRESS #####
//#######################
//The MAC address needs to be provided by the driver during initialisation.


//For further information please see the project technical manual





//*****************************
//*****************************
//********** DEFINES **********
//*****************************
//*****************************
#ifndef NIC_C_INIT		//Do only once the first time this file is used
#define	NIC_C_INIT

#include "minimac2.h"

//----- ETHERNET SPEED TO USE -----
#define	NIC_INIT_SPEED						1	//0 = allow speed 10 / 100 Mbps, 1 = force speed to 10 Mbps



//----- DATA TYPE DEFINITIONS -----
#define	ETHERNET_HEADER_LENGTH		(sizeof(struct ethernet_header))

void nic_initialise (BYTE init_config);
WORD nic_check_for_rx (void);
BYTE nic_ok_to_do_tx (void);
BYTE nic_read_next_byte (BYTE *data);
BYTE nic_read_array (BYTE *array_buffer, WORD array_length);
void nic_move_pointer (SIGNED_WORD move_pointer_to_ethernet_byte);
void nic_rx_dump_packet (void);
BYTE nic_setup_tx (void);
void nic_tx_writen_directly(WORD bytes_writen);
void nic_write_next_byte (BYTE data);
BYTE nic_write_array(BYTE *array_buffer, WORD array_length);
BYTE * nic_get_wrpointer();

void nic_write_tx_word_at_location (WORD byte_address, WORD data);
void write_eth_header_to_nic (MAC_ADDR *remote_mac_address, WORD ethernet_packet_type);
void nic_tx_packet (void);



extern BYTE nic_is_linked;
extern WORD nic_tx_len;
extern BYTE nic_rx_packet_waiting_to_be_dumped;

#endif








