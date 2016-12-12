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

#include "sha1.h"
#include "base64.h"

#include "prog_timer.h"

#include "websoc_server.h"

#ifndef WEBSOC_SERVER_PORT
#warning "WEBSOC_SERVER_PORT not defined, assume 3897"
#define WEBSOC_SERVER_PORT      (3897)
#endif

#define TCP_TIMEOUT             10000
#define TCP_TIMEOUT_TICK        (TCP_TIMEOUT/10)

#define CONST_STR_LEN(str)      (sizeof(str) - 1)

enum enWEBSOCServer_sm {
    SM_OPEN_SOCKET,
    SM_WAIT_FOR_CONNECTION,
    SM_PROCESS_CONNECTION,
    SM_TX_HANDSHAKE_RESPONSE,
    SM_TX_RESPONSE
};

#if 1
static const char websoc_ansver_part1[] =
"HTTP/1.1 101 Web Socket Protocol Handshake\r\
Upgrade: WebSocket\r\
Connection: Upgrade\r\
Sec-WebSocket-Accept: ";
#else
static const char websoc_ansver_part1[] =
"HTTP/1.1 101 Web Socket Protocol Handshake\r\
Upgrade: WebSocket\r\
Connection: Upgrade\r\
WebSocket-Origin: http://localhost:8888\r\
WebSocket-Location: ws://localhost:9876/\r\
Sec-WebSocket-Accept: ";
#endif


static const char websoc_ansver_part2[] =
"WebSocket-Protocol: sample\r\n\r\n";

static const char websoc_magick[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static const char websoc_key_str[] = "Sec-WebSocket-Key: ";

static uint32_t tcp_server_socket_timeout_timer;
static progtimer_desc_t tcp_timer;

void websoc_timeout_timer_cb(void* p) {
    if (tcp_server_socket_timeout_timer)
        tcp_server_socket_timeout_timer -= TCP_TIMEOUT_TICK;
}

void process_websoc_server() {
    static BYTE our_tcp_server_socket = TCP_INVALID_SOCKET;
    static BYTE our_tcp_server_state = SM_OPEN_SOCKET;

    BYTE data;
    static BYTE result_key[29]; // gJomYRcKbXDXWqpzypIPbhOZgh4=
#if 0
    BYTE array_buffer[4];
#endif

    if (!tcp_timer) {
        tcp_timer = progtimer_new(TCP_TIMEOUT_TICK, websoc_timeout_timer_cb, NULL);
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
        our_tcp_server_socket = tcp_open_socket_to_listen(WEBSOC_SERVER_PORT);
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
            tcp_request_disconnect_socket(our_tcp_server_socket); // soft close
            //THERE HAS BEEN NO COMMUNICATIONS FROM CLIENT TIMEOUT
            //RESET SOCKET AS WE ASSUME CLIENT HAS BEEN LOST
            //tcp_close_socket(our_tcp_server_socket); // har close
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

            BYTE websoc_client_key[24]; //SNQkjtF7mjPrc5mwhMSafw==
            // finding "Sec-WebSocket-Key:"
            BYTE* pMacher = (BYTE*)websoc_key_str;
            BYTE  key_found = FALSE;
            while(tcp_read_next_rx_byte(&data) != 0) {
                if (*pMacher == data) {
                    ++pMacher;
                    if (pMacher - (BYTE*)websoc_key_str == CONST_STR_LEN(websoc_key_str)) {
                        key_found = TRUE;
                        break;
                    }
                } else {
                    pMacher = (BYTE*)websoc_key_str;
                }
            }

            if (key_found &&
                   (0 != tcp_read_rx_array(websoc_client_key, sizeof(websoc_client_key)))) {
                SHA1_CTX ctx;
                BYTE hash_out[20];


                SHA1Init(&ctx);
#if  1
                SHA1Update(&ctx, websoc_client_key, sizeof(websoc_client_key));
                SHA1Update(&ctx, (const unsigned char *)websoc_magick, CONST_STR_LEN(websoc_magick));
#else // test, mast be 661295c9cbf9d6b2f6428414504a8deed3020641
                static const char test_Str[] = "test string";
                SHA1Update(&ctx, test_Str, CONST_STR_LEN(test_Str));
#endif
                SHA1Final(hash_out, &ctx);

                Base64encode((char*)result_key, (const char*)hash_out, sizeof(hash_out));

                our_tcp_server_state = SM_TX_HANDSHAKE_RESPONSE;
            } else {
                //SEND RESPONSE
                //our_tcp_server_state = SM_TX_RESPONSE;
            }
            tcp_dump_rx_packet();
        }
#if 1
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
    case SM_TX_HANDSHAKE_RESPONSE:
        //----- TX RESPONSE -----
        if (!tcp_setup_socket_tx(our_tcp_server_socket))
        {
            //Can't tx right now - try again next time
            break;
        }
        tcp_write_array((BYTE*)websoc_ansver_part1, CONST_STR_LEN(websoc_ansver_part1));
        tcp_write_array(result_key, sizeof(result_key) - 1);
        tcp_write_next_byte('\n');
        tcp_write_array((BYTE*)websoc_ansver_part2, CONST_STR_LEN(websoc_ansver_part2));

        //SEND THE PACKET
        tcp_socket_tx_packet(our_tcp_server_socket);
        our_tcp_server_state = SM_TX_RESPONSE;
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
