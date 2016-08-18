#****************************************************************************
#* CMakeLists.txt
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

cmake_minimum_required(VERSION 2.8.5)

project(or1k-spartan6-platform_bootloader ASM C)

# указать путь до модулей
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

set(LD_SCRIPT_FILE_IN  "${CMAKE_CURRENT_SOURCE_DIR}/link.ld.in")
set(LD_SCRIPT_FILE     "${CMAKE_CURRENT_BINARY_DIR}/link.ld")

math(EXPR MAIN_MEMORY_SIZE_B "${FPGA_DP_MEMORY_USE} * 16 * 1024 / 8")
configure_file(${LD_SCRIPT_FILE_IN} ${LD_SCRIPT_FILE} @ONLY)

INCLUDE("or1k_toolchain")

add_subdirectory(gdb_stub)
SET(BOOT_LIB_SIM_NOTRAP	    gdb_stub_sim_notrap)
if(DEBUGER_AT_BOOTLOADER)
    SET(BOOT_LIB    gdb_stub)
else()
    SET(BOOT_LIB    gdb_stub_notrap)
endif()

add_definitions(-DBOOTLOADER)

# TESTS
include_directories(gdb_stub)
set(TEST_SRC    freqmeters.c)
add_definitions(
    -DSTART_FREQMETER
    -DFREQMETERS_COUNT=24
    -DF_REF=${REFERENCE_CLOCK_HZ})
#

add_executable(${PROJECT_NAME}.elf
    ${BOOT_S}
    )
target_link_libraries(${PROJECT_NAME}.elf
    ${BOOT_LIB}
    )

add_executable(${PROJECT_NAME}_sim.elf
    ${BOOT_S}
    ${TEST_SRC}
    test.c
    )
target_link_libraries(${PROJECT_NAME}_sim.elf
    ${BOOT_LIB_SIM_NOTRAP}
    )

function(elf2hex ELF_FILE HEX_FILE)
    # make listing
    add_custom_command(
	OUTPUT  ${ELF_FILE}.lst
	DEPENDS ${ELF_FILE}
	COMMAND ${CMAKE_OBJDUMP} -h -d -S ${ELF_FILE} > ${ELF_FILE}.lst
	)

    # make binary
    add_custom_command(
	OUTPUT  ${ELF_FILE}.bin
	DEPENDS ${ELF_FILE} ${ELF_FILE}.lst
	COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${ELF_FILE}.bin
	)

    #product
    add_custom_command(
	OUTPUT  ${HEX_FILE}
	DEPENDS ${ELF_FILE}.bin
	COMMAND
	    xxd -ps -c 4 ${ELF_FILE}.bin > ${HEX_FILE}
	)


endfunction()

set(bootloader_HEX	    ${CMAKE_CURRENT_BINARY_DIR}/bootloader.hex)
set(bootloader_notrap_HEX   ${CMAKE_CURRENT_BINARY_DIR}/bootloader_notrap.hex)

elf2hex(${PROJECT_NAME}.elf ${bootloader_HEX})
elf2hex(${PROJECT_NAME}_sim.elf ${bootloader_notrap_HEX})

add_custom_target(bootloader DEPENDS ${bootloader_HEX} ${bootloader_notrap_HEX})

SET(BOOTLOADER_IMAGE_ELF        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf         PARENT_SCOPE)
SET(BOOTLOADER_IMAGE		${bootloader_HEX}	    PARENT_SCOPE)
SET(BOOTLOADER_SIM_IMAGE	${bootloader_notrap_HEX}    PARENT_SCOPE)
