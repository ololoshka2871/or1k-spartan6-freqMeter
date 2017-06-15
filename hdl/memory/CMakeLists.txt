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

SET(SYSTEM_MEMORY_FILE_NAME_IN      ${CMAKE_CURRENT_SOURCE_DIR}/wb_dp_ram_primitive.v.in)
SET(SYSTEM_MEMORY_BLOCK_TEMPLATE    ${CMAKE_CURRENT_SOURCE_DIR}/memory_block_template.v.in)

SET(SYSTEM_MEMORY_FILE_NAME     wb_dp_ram_primitive.v)
SET(SYSTEM_MEMORY_MODULE	${CMAKE_CURRENT_BINARY_DIR}/${SYSTEM_MEMORY_FILE_NAME})
SET(SYSTEM_MEMORY_MODULE_SIM	${CMAKE_CURRENT_BINARY_DIR}/sim_${SYSTEM_MEMORY_FILE_NAME})

################################################################################

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
        xxd -ps -c 4 ${SYSTEM_MEMORY_BLOCK_TEMPLATE} > ${TEST_IMAGE_FOR_DMA_MEMORY_NAME}
    )
endif()

#getnerate count image
set(COUNT_TEST_MEMORY_IMAGE     ${CMAKE_CURRENT_BINARY_DIR}/count.hex)
set(MKKOUNT_SCRIPT              ${SCRIPTS_DIR}/mkcount.sh)
add_custom_command(
    OUTPUT ${COUNT_TEST_MEMORY_IMAGE}
    COMMAND
        ${MKKOUNT_SCRIPT} 2048 > ${COUNT_TEST_MEMORY_IMAGE}
    )

################################################################################

set(MK_SYSMEM_MODULE            ${SCRIPTS_DIR}/mk_system_memory_module.py)

add_custom_command(
    OUTPUT ${SYSTEM_MEMORY_MODULE}
    COMMAND
        ${PYTHON_EXECUTABLE} ${MK_SYSMEM_MODULE}
            --templatemodule ${SYSTEM_MEMORY_FILE_NAME_IN}
            --templateinstance ${SYSTEM_MEMORY_BLOCK_TEMPLATE}
            --image ${BOOTLOADER_IMAGE}
            --blocks ${SYSTEM_FPGA_BMEMORY_USE}
            --blocksize ${MEMORY_UNIT_SIZE_BYTES} > ${SYSTEM_MEMORY_MODULE}
    DEPENDS
        bootloader
        ${BOOTLOADER_IMAGE}
        ${SYSTEM_MEMORY_FILE_NAME_IN}
        ${SYSTEM_MEMORY_BLOCK_TEMPLATE}
        ${COUNT_TEST_MEMORY_IMAGE} # need to success build
        ${MK_SYSMEM_MODULE}
    COMMENT
        "Making system memory image"
    )

add_custom_target(memory_refresh DEPENDS ${SYSTEM_MEMORY_MODULE})

add_custom_command(
    OUTPUT ${SYSTEM_MEMORY_MODULE_SIM}
    COMMAND
        ${PYTHON_EXECUTABLE} ${MK_SYSMEM_MODULE}
            --templatemodule ${SYSTEM_MEMORY_FILE_NAME_IN}
            --templateinstance ${SYSTEM_MEMORY_BLOCK_TEMPLATE}
            --image ${BOOTLOADER_SIM_IMAGE}
            --blocks ${SYSTEM_FPGA_BMEMORY_USE}
            --blocksize ${MEMORY_UNIT_SIZE_BYTES} > ${SYSTEM_MEMORY_MODULE_SIM}
    DEPENDS
        bootloader
        ${BOOTLOADER_SIM_IMAGE}
        ${SYSTEM_MEMORY_FILE_NAME_IN}
        ${SYSTEM_MEMORY_BLOCK_TEMPLATE}
        ${MK_SYSMEM_MODULE}
    COMMENT
        "Making test system memory image"
    )

add_custom_target(memory_refresh_sim DEPENDS ${SYSTEM_MEMORY_MODULE_SIM})

################################################################################

SET(MEMORY_HDL_V	${RAM_HDL_V};${SYSTEM_MEMORY_MODULE})
SET(MEMORY_HDL_SIM_V	${RAM_HDL_V};${SYSTEM_MEMORY_MODULE_SIM})

add_custom_target(memory_sources
    SOURCES
        ${RAM_HDL_V} ${SYSTEM_MEMORY_FILE_NAME_IN}
        ${SYSTEM_MEMORY_BLOCK_TEMPLATE}
    )

set(MEMORY_HDL_V	${MEMORY_HDL_V}		PARENT_SCOPE)
set(MEMORY_HDL_SIM_V	${MEMORY_HDL_SIM_V}	PARENT_SCOPE)
set(TEST_IMAGE_FOR_DMA_MEMORY_NAME ${TEST_IMAGE_FOR_DMA_MEMORY_NAME} PARENT_SCOPE)
set(COUNT_TEST_MEMORY_IMAGE ${COUNT_TEST_MEMORY_IMAGE} PARENT_SCOPE)
