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

cmake_minimum_required(VERSION 2.8.11)

set(NANOPB_DIST             ${CMAKE_CURRENT_SOURCE_DIR}/nanopb-dist)

set(CMAKE_MODULE_PATH       ${CMAKE_MODULE_PATH} "${NANOPB_DIST}/extra")

find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})

if (PYTHON_VERSION_MAJOR STRGREATER "2")
    message(STATUS "Forsing python2 for nanopb")
    find_program(PYTHON_2   python2)
    if (NOT PYTHON_2)
        message(STATUS "python2 not found, U can try use python3, but it was not works for me")
    else()
        set(PYTHON_EXECUTABLE ${PYTHON_2})
    endif()
endif()

set(PROTOBUF_PROTO_FILE     ${CMAKE_CURRENT_SOURCE_DIR}/protocol.proto)
get_filename_component(PROTO_FILE ${PROTOBUF_PROTO_FILE} NAME_WE)
set(PROTOBUF_OPTIONS_FILE   ${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_FILE}.options)

add_custom_target(protobuf_proto_def SOURCES ${PROTOBUF_PROTO_FILE} ${PROTOBUF_OPTIONS_FILE})

set(PROTOBUF_PROTO_FILE_W   ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILE}.proto)
set(PROTOBUF_OPTIONS_FILE_W ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILE}.options)

configure_file(${PROTOBUF_PROTO_FILE} ${PROTOBUF_PROTO_FILE_W} @ONLY)
configure_file(${PROTOBUF_OPTIONS_FILE} ${PROTOBUF_OPTIONS_FILE_W} @ONLY)


nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOBUF_PROTO_FILE_W})

set(PROTOCOL_SRC
    protobuf-protocol.c
    )

set(_NANOPB_INCLUDE_DIRS
        ${CMAKE_CURRENT_BINARY_DIR}
        ${NANOPB_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

add_definitions(
    -DPB_FIELD_16BIT
#    -DPB_NO_ERRMSG
    )

include_directories(${_NANOPB_INCLUDE_DIRS} ${LIBHW_INCLUDE_DIR} ${IBEXUK_IP_INCLUDE_DIRS})

add_library(nanopb STATIC ${NANOPB_SRCS} ${PROTO_SRCS} ${PROTOCOL_SRC})

set(NANOPB_INCLUDE_DIRS   ${_NANOPB_INCLUDE_DIRS}     PARENT_SCOPE)
set(PROTOBUF_PROTO_FILE   ${PROTOBUF_PROTO_FILE_W}    PARENT_SCOPE)
