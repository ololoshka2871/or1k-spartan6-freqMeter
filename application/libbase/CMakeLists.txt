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

SET(LIBBASE_SOURCES
    mem_managment.c
    assert_trap.c
    printf-warper.c
    graycode.c
    bcd.c
    sha1.c
    base64.c
    )

include_directories(.)

add_definitions(
    -DSYSTEM_HEAP_SIZE=${SYSTEM_HEAP_SIZE}
    -DMACTX_HEAP_BASE=${MAC_TX_MEM_BASE}
    -DMACTX_HEAP_SIZE=${MAC_TX_MEM_SIZE}
    -DBYTE_ORDER=__BYTE_ORDER__
    -DLITTLE_ENDIAN=__ORDER_LITTLE_ENDIAN__
    -DBIG_ENDIAN=__ORDER_BIG_ENDIAN__
    )

add_library(base STATIC ${LIBBASE_SOURCES})

set(APP_STARTUP_SRC      ${CMAKE_CURRENT_SOURCE_DIR}/startup.c   PARENT_SCOPE)
set(LIBBASE_INCLUDE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}             PARENT_SCOPE)
