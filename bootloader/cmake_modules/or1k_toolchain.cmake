#****************************************************************************
#* cmake_modules/or1k_toolchain.cmake
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
#* 3. Neither the name NuttX nor the names of its contributors may be
#*    used to endorse or promote products derived from this software
#*    without specific prior written permission.
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


INCLUDE(CMakeForceCompiler)

MESSAGE(STATUS "Setting toolchain or1k-elf-")

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

SET(TOOLCHAIN_PREFIX	or1k-elf-)

# specify the cross compiler
CMAKE_FORCE_C_COMPILER(${TOOLCHAIN_PREFIX}gcc GNU)
SET(CMAKE_LINKER ${TOOLCHAIN_PREFIX}gcc)
SET(CMAKE_C_LINK_EXECUTABLE
    "<CMAKE_LINKER> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
SET(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
SET(CMAKE_ASM_COMPILE_OBJECT
    "<CMAKE_ASM_COMPILER> <FLAGS> -c <SOURCE> -o <OBJECT>")

SET(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
SET(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)

# linker script file
set(LD_SCRIPT_FILE_IN  "${CMAKE_CURRENT_SOURCE_DIR}/link.ld.in")
set(LD_SCRIPT_FILE     "${CMAKE_CURRENT_SOURCE_DIR}/link.ld")
set(MEM_BASE 0x10000000)

SET(COMMON_FLAGS "-msoft-float -std=gnu99 -mno-delay")

SET(CMAKE_C_FLAGS_COMMON "\
    -Ttext ${MEM_BASE} \
    -Wall \
    -pipe \
    -ffunction-sections -fdata-sections \
    -msoft-div -msoft-mul -mno-ror -mno-cmov -mno-sext \
    ${COMMON_FLAGS}"
    )

add_definitions(-D__OR1K_NODELAY__ -D__OR1K__)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_COMMON} -g -Os")
set(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_COMMON} -Os")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_COMMON} -g -Os")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_COMMON} -Os")

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

SET(CMAKE_ASM_FLAGS "-mno-delay -Wa,--defsym,__OR1K_NODELAY__=1")

set(CMAKE_EXE_LINKER_FLAGS "\
    ${COMMON_FLAGS} \
    -nostartfiles \
    -T${LD_SCRIPT_FILE} \
    -Wl,-gc-sections"
    )

#-nodefaultlibs -nostdlib
#-Wl,--whole-archive \
