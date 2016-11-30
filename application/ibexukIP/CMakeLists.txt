#****************************************************************************
#*
#*   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
#*   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions
#* are met:
#*
#* 1. Redistributions of source code must retain the above copyright
#*    notice, this list of conditions and the following disclaimer.
#* 2. Redistributions in binary form must reproduce the above copyright
#*    notice, this list of conditions and the following disclaimer in
#*    the documentation and/or other materials provided with the
#*    distribution.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#* POSSIBILITY OF SUCH DAMAGE.
#*
#****************************************************************************/

cmake_minimum_required(VERSION 2.8.5)

set(CONFIG_FILE_IN       ${CMAKE_CURRENT_SOURCE_DIR}/main.h.in)
set(CONFIG_FILE          ${CMAKE_CURRENT_BINARY_DIR}/main.h)
configure_file(${CONFIG_FILE_IN} ${CONFIG_FILE} @ONLY)


set(ETH_CONFIG_FILE_IN   ${CMAKE_CURRENT_SOURCE_DIR}/eth-main.h.in)
set(ETH_CONFIG_FILE      ${CMAKE_CURRENT_BINARY_DIR}/eth-main.h)

if (SERVER_HTTP)
    set(STACK_USE_HTTP_STR "#define STACK_USE_HTTP")
    set(STACK_USE_TCP_STR  "#define STACK_USE_TCP")
else()
    set(STACK_USE_HTTP_STR "// Macro STACK_USE_HTTP is not defined")
    if (SERVER_WEBSOC)
        set(STACK_USE_TCP_STR  "#define STACK_USE_TCP")
    else()
        set(STACK_USE_TCP_STR "// Macro STACK_USE_TCP is not defined")
    endif()
endif()

configure_file(${ETH_CONFIG_FILE_IN} ${ETH_CONFIG_FILE} @ONLY)

set(IBEXUK_IP_INCLUDE_DIRS
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

include_directories(${IBEXUK_IP_INCLUDE_DIRS}
    ${LIBHW_INCLUDE_DIR}
    ${LIBBASE_INCLUDE_DIR})

set(IBEXUK_IP_SRC
    eth-nic.c
    timers.c

    eth-arp.c
    eth-icmp.c
    eth-ip.c
    eth-main.c
    eth-udp.c
    eth-dhcp.c
    )

if (SERVER_HTTP)
    list(APPEND IBEXUK_IP_SRC eth-http.c eth-tcp.c)
else()
    if (SERVER_WEBSOC)
        list(APPEND IBEXUK_IP_SRC eth-tcp.c)
    endif()
endif()

set(difenitions
    GROUP_WRITES=1
    HTTP_MAX_FILENAME_LENGTH=${SITE_FILENAME_MAX}
    HTTP_MAX_FILEEXT_LENGTH=${SITE_FILEEXT_MAX}
    )

if(ETHERNET_USE_DHCP)
    list(APPEND difenitions DHCP_ON_STARTUP)
endif()

if(ETHERNET_SKIP_UDP_CHECKSUMS)
    list(APPEND difenitions SKIP_UDP_CHECKSUMS)
endif()

add_library(ibexukIP STATIC ${IBEXUK_IP_SRC})

set_target_properties(ibexukIP PROPERTIES COMPILE_DEFINITIONS "${difenitions}")

set(IBEXUK_IP_INCLUDE_DIRS  ${IBEXUK_IP_INCLUDE_DIRS}       PARENT_SCOPE)
