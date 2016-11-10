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

//----- ETHERNET SPEED TO USE -----
#define	NIC_INIT_SPEED						1	//0 = allow speed 10 / 100 Mbps, 1 = force speed to 10 Mbps



//----- DATA TYPE DEFINITIONS -----
typedef struct _ETHERNET_HEADER
{
    MAC_ADDR		destination_mac_address;
    MAC_ADDR		source_mac_address;
    WORD_VAL		type;
} __attribute__((packed)) ETHERNET_HEADER;
#define	ETHERNET_HEADER_LENGTH		(sizeof(ETHERNET_HEADER))


extern BYTE nic_is_linked;
extern WORD nic_tx_len;
extern BYTE nic_rx_packet_waiting_to_be_dumped;

#endif








