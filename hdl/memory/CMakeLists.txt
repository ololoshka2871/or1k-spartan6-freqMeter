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

cmake_minimum_required(VERSION 3.0.2)

file(GLOB_RECURSE
    RAM_HDL_V
    FOLLOW_SYMLINKS
    "*.v"
    )

SET(SYSTEM_MEMORY_FILE_NAME	wb_dp_ram.v)

list(REMOVE_ITEM RAM_HDL_V      ${CMAKE_CURRENT_SOURCE_DIR}/${SYSTEM_MEMORY_FILE_NAME})

SET(SYSTEM_MEMORY_MODULE	${CMAKE_CURRENT_BINARY_DIR}/${SYSTEM_MEMORY_FILE_NAME})
SET(SYSTEM_MEMORY_MODULE_SIM	${CMAKE_CURRENT_BINARY_DIR}/sim_${SYSTEM_MEMORY_FILE_NAME})
SET(SYSTEM_MEMORY_MODULE_IN	${CMAKE_CURRENT_SOURCE_DIR}/${SYSTEM_MEMORY_FILE_NAME})

set(IMAGE   ${BOOTLOADER_IMAGE})
configure_file(${SYSTEM_MEMORY_MODULE_IN} ${SYSTEM_MEMORY_MODULE}.tmp @ONLY)
set(IMAGE   ${BOOTLOADER_SIM_IMAGE})
configure_file(${SYSTEM_MEMORY_MODULE_IN} ${SYSTEM_MEMORY_MODULE_SIM}.tmp @ONLY)

#force rebuild, if bootloader changed
add_custom_target(memory_refresh
    COMMAND
        ${SCRIPTS_DIR}/checkmemory.sh ${SYSTEM_MEMORY_MODULE}.tmp
            ${SYSTEM_MEMORY_MODULE} ${BOOTLOADER_IMAGE}
    DEPENDS
        bootloader ${SYSTEM_MEMORY_MODULE}.tmp test_image.hex
    COMMENT
        "Checking main memory image"
    )
add_custom_target(memory_refresh_sim
    COMMAND
	${SCRIPTS_DIR}/checkmemory.sh ${SYSTEM_MEMORY_MODULE_SIM}.tmp
	    ${SYSTEM_MEMORY_MODULE_SIM} ${BOOTLOADER_SIM_IMAGE}
    DEPENDS
        bootloader ${SYSTEM_MEMORY_MODULE_SIM}.tmp test_image.hex
    COMMENT
	"Checking sim memory image"
    )

set(TEST_IMAGE_FOR_DMA_MEMORY_NAME  ${CMAKE_CURRENT_BINARY_DIR}/test_image.hex)

# Чтобы затестить готовый пакет а не рандомные данные положить данные
# в этот файл текстом для команды verilog $readmemh()
set(TEST_POCKET_FILE                ${CMAKE_CURRENT_SOURCE_DIR}/tx_data.raw)
if (EXISTS ${TEST_POCKET_FILE})
add_custom_command(
    OUTPUT  ${TEST_IMAGE_FOR_DMA_MEMORY_NAME}
    DEPENDS memory_refresh
    COMMAND
        cp ${TEST_POCKET_FILE} ${TEST_IMAGE_FOR_DMA_MEMORY_NAME}
    )
else()
#generate test image for memory
add_custom_command(
    OUTPUT  ${TEST_IMAGE_FOR_DMA_MEMORY_NAME}
    DEPENDS memory_refresh
    COMMAND
        xxd -ps -c 4 ${SYSTEM_MEMORY_MODULE} > ${TEST_IMAGE_FOR_DMA_MEMORY_NAME}
    )
endif()

SET(MEMORY_HDL_V	${RAM_HDL_V};${SYSTEM_MEMORY_MODULE})
SET(MEMORY_HDL_SIM_V	${RAM_HDL_V};${SYSTEM_MEMORY_MODULE_SIM})

add_custom_target(memory_sources
    SOURCES
        ${SYSTEM_MEMORY_MODULE_IN} ${RAM_HDL_V}
    )

set(MEMORY_HDL_V	${MEMORY_HDL_V}		PARENT_SCOPE)
set(MEMORY_HDL_SIM_V	${MEMORY_HDL_SIM_V}	PARENT_SCOPE)
set(TEST_IMAGE_FOR_DMA_MEMORY_NAME ${TEST_IMAGE_FOR_DMA_MEMORY_NAME} PARENT_SCOPE)
