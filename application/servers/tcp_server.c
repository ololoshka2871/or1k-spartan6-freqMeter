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
#include "eth-tcp.h"

#include "prog_timer.h"

#include "tcp_server.h"

#define TCP_TIMEOUT             10000
#define TCP_TIMEOUT_TICK        (TCP_TIMEOUT/10)

enum enUDPServer_sm {
    SM_OPEN_SOCKET,
    SM_WAIT_FOR_CONNECTION,
    SM_PROCESS_CONNECTION,
    SM_TX_RESPONSE
};

static uint32_t tcp_server_socket_timeout_timer;
static progtimer_desc_t tcp_timer;

void tcp_timeout_timer_cb(void* p) {
    if (tcp_server_socket_timeout_timer)
        tcp_server_socket_timeout_timer -= TCP_TIMEOUT_TICK;
}

void process_tcp_server() {
    static BYTE our_tcp_server_socket = TCP_INVALID_SOCKET;
    static BYTE our_tcp_server_state = SM_OPEN_SOCKET;

#if 0
    BYTE data;
    BYTE array_buffer[4];
#endif

    if (!tcp_timer) {
        tcp_timer = progtimer_new(TCP_TIMEOUT_TICK, tcp_timeout_timer_cb, NULL);
    }

    if (!nic_linked_and_ip_address_valid)
    {
        //----- WE ARE NOT CONNECTED OR DO NOT YET HAVE AN IP ADDRESS -----
        our_tcp_server_state = SM_OPEN_SOCKET;
        //Ensure our socket is closed if we have just lost the Ethernet connection
        tcp_close_socket_from_listen(our_tcp_server_socket);
        return; //Exit as we can't do anything without a connection
    }

    switch (our_tcp_server_state)
    {
    case SM_OPEN_SOCKET:
        //----- OPEN SOCKET -----
        //We will listen on port 8923 (change as required)
        //We shouldn't have a socket currently, but make sure
        if (our_tcp_server_socket != TCP_INVALID_SOCKET)
            tcp_close_socket(our_tcp_server_socket);
        our_tcp_server_socket = tcp_open_socket_to_listen(8923);
        if (our_tcp_server_socket != TCP_INVALID_SOCKET)
        {
            our_tcp_server_state = SM_WAIT_FOR_CONNECTION;
            break;
        }
        //Could not open a socket - none currently available - keep trying
        break;
    case SM_WAIT_FOR_CONNECTION:
        //----- WAIT FOR A CLIENT TO CONNECT -----
        if(tcp_is_socket_connected(our_tcp_server_socket))
        {
            //A CLIENT HAS CONNECTED TO OUR SOCKET
            our_tcp_server_state = SM_PROCESS_CONNECTION;
            //Set our client has been lost timeout (to avoid client
            //disappearing and causing this socket to never be closed)
            tcp_server_socket_timeout_timer = TCP_TIMEOUT;
        }
        break;
    case SM_PROCESS_CONNECTION:
        //----- PROCESS CLIENT CONNECTION -----
        if (tcp_server_socket_timeout_timer == 0)
        {
            //THERE HAS BEEN NO COMMUNICATIONS FROM CLIENT TIMEOUT
            //RESET SOCKET AS WE ASSUME CLIENT HAS BEEN LOST
            tcp_close_socket(our_tcp_server_socket);
            //As this socket is a server the existing connection will be closed
            //but the socket will be reset to wait for a new connection (use
            //tcp_close_socket_from_listen if you want to fully close it)
            our_tcp_server_state = SM_WAIT_FOR_CONNECTION;
        }
        if (tcp_check_socket_for_rx(our_tcp_server_socket))
        {
            //SOCKET HAS RECEIVED A PACKET - PROCESS IT
            tcp_server_socket_timeout_timer = TCP_TIMEOUT; //Reset our timeout timer
            //READ THE PACKET AS REQURIED
#if 0
            if (tcp_read_next_rx_byte(&data) == 0)
            {
                //Error - no more bytes in rx packet
            }
            //OR USE
            if (tcp_read_rx_array (array_buffer, sizeof(array_buffer)) == 0)
            {
                //Error - no more bytes in rx packet
            }
#endif
            //DUMP THE PACKET
            tcp_dump_rx_packet();
            //SEND RESPONSE
            our_tcp_server_state = SM_TX_RESPONSE;
        }
#if 0
        if (tcp_does_socket_require_resend_of_last_packet(our_tcp_server_socket))
        {
            //RE-SEND LAST PACKET TRANSMITTED
            //(TCP requires resending of packets if they are not acknowledged and to
            //avoid requiring a large RAM buffer the application needs to remember
            //the last packet sent on a socket so it can be resent if required).
            our_tcp_server_state = SM_TX_RESPONSE;
        }
#endif
        if(!tcp_is_socket_connected(our_tcp_server_socket))
        {
            //THE CLIENT HAS DISCONNECTED
            our_tcp_server_state = SM_WAIT_FOR_CONNECTION;
        }
        break;
    case SM_TX_RESPONSE:
        //----- TX RESPONSE -----
        if (!tcp_setup_socket_tx(our_tcp_server_socket))
        {
            //Can't tx right now - try again next time
            break;
        }
        //WRITE THE TCP DATA
        tcp_write_next_byte('H');
        tcp_write_next_byte('i');
        tcp_write_next_byte('\n');
        tcp_write_next_byte(0x00);
        //You can also use tcp_write_array()
        //SEND THE PACKET
        tcp_socket_tx_packet(our_tcp_server_socket);
        our_tcp_server_state = SM_PROCESS_CONNECTION;
        break;
    }
}
