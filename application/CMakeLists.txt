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

project(or1k-spartan6-platform_application ASM C)

INCLUDE("or1k_toolchain")

# указать путь до модулей
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

set(MK_APP_LD_SCRIPT   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/mkldscript.sh")
set(LD_SCRIPT_FILE_IN  "${CMAKE_CURRENT_SOURCE_DIR}/link.app.ld.in")
set(LD_SCRIPT_FILE     "${CMAKE_CURRENT_BINARY_DIR}/link.app.ld")

add_custom_target(mkldscript.files
    SOURCES
        ${MK_APP_LD_SCRIPT} ${LD_SCRIPT_FILE_IN}
    )

add_custom_command(
    OUTPUT
        ${LD_SCRIPT_FILE}
    COMMAND
        ${MK_APP_LD_SCRIPT}
            ${LD_SCRIPT_FILE_IN}
            ${BOOTLOADER_IMAGE_ELF}
            ${HEADER_W1}
            ${HEADER_W2}
            ${USER_CODE_FLASH_OFFSET}
            ${MAC_TX_MEM_BASE}
            ${MAC_RX_MEM_BASE}
            ${MYMINMAC_RX_SLOTS}
            ${MYMINMAC_TX_SLOTS}
            ${MEMORY_UNIT_SIZE}
            ${MTU}
            ${TOOLCHAIN_PREFIX} > ${LD_SCRIPT_FILE}
    DEPENDS
        ${BOOTLOADER_IMAGE_ELF} ${LD_SCRIPT_FILE_IN}
    COMMENT
        "Generating build script for application"
    )

INCLUDE("or1k_toolchain")

add_definitions(
    -DFREQMETERS_COUNT=${F_INPUTS_COUNT}
    -DF_REF=${REFERENCE_CLOCK_HZ}UL
    )

add_custom_target(mkldscript DEPENDS ${LD_SCRIPT_FILE})

add_subdirectory(hw)
add_subdirectory(lib)

include_directories(hw)

get_target_property(hwlib_difenitions hw COMPILE_DEFINITIONS)

set(APPLICATION_ELF_NAME app.elf)

add_executable(${APPLICATION_ELF_NAME}
    ${APP_STARTUP_SRC}
    main.c
    )
target_link_libraries(${APPLICATION_ELF_NAME}
    hw
    )
set_target_properties(${APPLICATION_ELF_NAME} PROPERTIES COMPILE_DEFINITIONS "${hwlib_difenitions}")
add_dependencies(${APPLICATION_ELF_NAME} mkldscript)

function(elf2bin ELF_FILE BIN_FILE)
    # make listing
    add_custom_command(
        OUTPUT  ${ELF_FILE}.lst
        DEPENDS ${ELF_FILE}
        COMMAND ${CMAKE_OBJDUMP} -h -d -S ${ELF_FILE} > ${ELF_FILE}.lst
        )

    # make binary
    add_custom_command(
        OUTPUT  ${BIN_FILE}
        DEPENDS ${ELF_FILE} ${ELF_FILE}.lst
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
        )

endfunction()

set(application_BIN	    ${CMAKE_CURRENT_BINARY_DIR}/app.bin)

elf2bin(${APPLICATION_ELF_NAME} ${application_BIN})

add_custom_target(application DEPENDS ${application_BIN})

SET(APPLICATION_BINARY		${application_BIN}	    PARENT_SCOPE)
