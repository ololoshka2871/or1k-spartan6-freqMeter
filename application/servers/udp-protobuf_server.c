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

#include <stddef.h>

#include "main.h"
#include "eth-main.h"
#include "eth-udp.h"

#include "protobuf-protocol.h"

#include "udp-protobuf_server.h"

#ifndef UDP_SERVER_PORT
#error "UDP_SERVER_PORT mast be defined!"
#endif

//RETURN THE SOCKET BACK TO BROADCAST READY TO RECEIVE FROM ANYONE AGAIN
#define SOCK_RESET(s)    udp_socket[(s)].remote_device_info.ip_address.Val = 0xffffffff

enum enProtobufServer_state {
    SM_OPEN_SOCKET,
    SM_PROCESS_SOCKET,
    SM_TX_RESPONSE,
    SM_TX_ERROR_MSG
};

void process_protobuf_server() {
    static BYTE our_udp_socket = UDP_INVALID_SOCKET;
    static enum enProtobufServer_state our_udp_server_state = SM_OPEN_SOCKET;

    if (!nic_linked_and_ip_address_valid)
    {
        //----- WE ARE NOT CONNECTED OR DO NOT YET HAVE AN IP ADDRESS -----
        our_udp_server_state = SM_OPEN_SOCKET;
        //Ensure our socket is closed if we have just lost the Ethernet connection
        udp_close_socket(&our_udp_socket);
        return; //Exit as we can't do anything without a connection
    }

    switch (our_udp_server_state) {
    case SM_OPEN_SOCKET:
        //----- OPEN SOCKET -----
        //(Leave device_info as null to setup to receive from anyone, remote_port
        //can be anything for rx)
        our_udp_socket = udp_open_socket(NULL, UDP_SERVER_PORT, 0);
        if (our_udp_socket != UDP_INVALID_SOCKET)
        {
            our_udp_server_state = SM_PROCESS_SOCKET;
            break;
        }
        //Could not open a socket - none currently available - keep trying
        break;
    case SM_PROCESS_SOCKET:
        //----- PROCESS SOCKET -----
        if (udp_check_socket_for_rx(our_udp_socket))
        {
            enum enProtobufServer_state newstate;
            if (protobuf_handle_request(udp_read_rx_array) == PB_OK)
                newstate = SM_TX_RESPONSE;
            else
                newstate = SM_TX_ERROR_MSG;

            //DUMP THE PACKET
            udp_dump_rx_packet();
            //SEND RESPONSE
            our_udp_server_state = newstate;
        }
        break;
    case SM_TX_RESPONSE:
        //----- TX RESPONSE -----
        //SETUP TX
        if (!udp_setup_tx(our_udp_socket))
        {
            //Can't tx right now - try again next time
            //Return the socket back to broadcast ready to receive from anyone again
            //Only enable the line below if you are broadcasting responses and
            //don't want to miss incoming packets to this socket from other devices
            //udp_socket[our_udp_socket].remote_device_info.ip_address.val = 0xffffffff;
            break;
        }
        //WRITE THE UDP DATA
        udp_write_next_byte('O');
        udp_write_next_byte('K');
        udp_write_next_byte('\n');
        udp_write_next_byte(0x00);
        //You can also use udp_write_array()
        //SEND THE PACKET
        udp_tx_packet();

        SOCK_RESET(our_udp_socket);
        our_udp_server_state = SM_PROCESS_SOCKET;
        break;
    case SM_TX_ERROR_MSG:
        if (!udp_setup_tx(our_udp_socket)) {
            break;
        }
        udp_write_next_byte('E');
        udp_write_next_byte('r');
        udp_write_next_byte('r');
        udp_write_next_byte('o');
        udp_write_next_byte('r');
        udp_write_next_byte('\n');
        udp_write_next_byte(0x00);

        udp_tx_packet();

        SOCK_RESET(our_udp_socket);
        our_udp_server_state = SM_PROCESS_SOCKET;
        break;
    } //switch (our_udp_server_state)
}
