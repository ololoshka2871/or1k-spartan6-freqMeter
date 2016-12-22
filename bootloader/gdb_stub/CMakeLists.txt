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

SET(GDB_HW_SRC		    gdb_hw.c gdb.c boot_spi.c)
SET(GDB_NORMAL_SRC	    gdb_main.c)
SET(GDB_NOTRAP_SRC	    gdb_main_notrap.c)
SET(GDB_NOTRAP_SIM_SRC	    gdb_main_sim_notrap.c)


if(PERIPHERIAL_ENABLE_HW_MUL)
    list(APPEND GDB_HW_SRC __mulsi3.c)
endif()

INCLUDE_DIRECTORIES(.)

add_definitions(
    -DSYSTEM_HEADER_W1=${SYSTEM_HEADER_W1}
    -DSYSTEM_HEADER_W2=${SYSTEM_HEADER_W2}
    -DSPI_MODULE_TINY
    )

add_library(gdb_stub STATIC ${GDB_HW_SRC} ${GDB_NORMAL_SRC})
add_library(gdb_stub_notrap STATIC ${GDB_HW_SRC} ${GDB_NOTRAP_SRC})
add_library(gdb_stub_sim_notrap STATIC ${GDB_HW_SRC} ${GDB_NOTRAP_SIM_SRC})

set(BOOT_S          ${CMAKE_CURRENT_SOURCE_DIR}/boot.S	PARENT_SCOPE)
set(GDB_STUB_DIR    ${CMAKE_CURRENT_SOURCE_DIR}         PARENT_SCOPE)
