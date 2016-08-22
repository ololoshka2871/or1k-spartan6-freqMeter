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

project(or1k-spartan6-FreqMeter NONE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(PythonInterp 3 REQUIRED)

add_subdirectory(tools)

# указать путь до модулей
set(CMAKE_MODULE_PATH
	${CMAKE_MODULE_PATH}
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include("halpers")

find_package(Xilinx REQUIRED)

#-----------------------------------config--------------------------------------

set(PART_NAME               "xc6slx9-3-tqg144"
    CACHE STRING "Xilinx FPGA model to synthesis")
set(INPUT_CLOCK_HZ          48000000
    CACHE STRING "FPGA main clock source (MHz)")
set(SPI_FLASH_CHIP          "W25Q32BV"
    CACHE STRING "SPI flash memory part name")
set(FPGA_DP_MEMORY_USE      32
    CACHE STRING "Number of RAMB16BWERs to use as main memory")

set(HEADER_W1               0x03030303
    CACHE STRING "Application syncronisation word 1")
set(HEADER_W2               0x1f1f1f1f
    CACHE STRING "Application syncronisation word 2")

set(DEBUGER_AT_BOOTLOADER   OFF
    CACHE BOOL   "Start debuger in bootloader")

#--- peripherial ----
set(PERIPHERIAL_ENABLE_GPIO OFF
    CACHE BOOL  "Enable GPIO IPCore at 0x12000500")
set(PERIPHERIAL_ENABLE_SEG7 OFF
    CACHE BOOL  "Enable SEG7 indicator IPCore at 0x12000400")
set(PERIPHERIAL_ENABLE_TIMER ON
    CACHE BOOL  "Enable UART1 IPCore at 0x12000100")
set(PERIPHERIAL_ENABLE_UART0 ON
    CACHE BOOL  "Enable UART1 IPCore at 0x12000000")
set(PERIPHERIAL_ENABLE_UART1 OFF
    CACHE BOOL  "Enable UART1 IPCore at 0x12000700")
set(PERIPHERIAL_ENABLE_ETHERNET OFF
    CACHE BOOL  "Enable ETHERNET IPCore 0x")
#--- /peripherial ---

#--- UARTs ----
set(UART0_BAUD              115200
    CACHE STRING "UART0 (debug) speed")
set(UART1_BAUD              921600
    CACHE STRING "UART1 speed")
#--- /UARTs ---

set(MASER_FREQ_COUNTER_LEN  30
    CACHE STRING "Master clock counter length (in bits)")
set(INPUT_FREQ_COUNTER_LEN  24
    CACHE STRING "Input clock counter length (in bits)")
set(TEST_SIGNAL_EVERY_MS    1000
    CACHE STRING "Test input signal present aproximaly every")

set(PLL_MULTIPLYER          1
    CACHE STRING "Input clock multiplier")

set(REFERENCE_CLOCK_DEVIDER 1
    CACHE STRING "Master clock devider")

set(CPU_CLOCK_DEVIDER       1
    CACHE STRING "CPU clock devider")

#----------------------------------/config--------------------------------------

#--------------------------------Calculable-------------------------------------

math(EXPR PLL_CLOCK_HZ
    "${INPUT_CLOCK_HZ} * ${PLL_MULTIPLYER}")

math(EXPR REFERENCE_CLOCK_HZ
    "${PLL_CLOCK_HZ} / ${REFERENCE_CLOCK_DEVIDER}")

math(EXPR CPU_CLOCK_HZ
    "${PLL_CLOCK_HZ} / ${CPU_CLOCK_DEVIDER}")

math(EXPR INPUT_CLOCK_PERIOD_NS_F_m100
    "1000000000 / (${INPUT_CLOCK_HZ} / 100)")


floterise(INPUT_CLOCK_PERIOD_NS_F ${INPUT_CLOCK_PERIOD_NS_F_m100} 100)
message(STATUS
    "Input clock = ${INPUT_CLOCK_HZ}Hz, period = ${INPUT_CLOCK_PERIOD_NS_F}ns")

set(calc_best_signal_test_devider ${tools_DIR}/calc_best_signal_test_devider.py)
execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} ${calc_best_signal_test_devider}
            --mindev=1
            --maxdev=${MASER_FREQ_COUNTER_LEN}
            --freq=${REFERENCE_CLOCK_HZ}
            --target=${TEST_SIGNAL_EVERY_MS}
    OUTPUT_VARIABLE TEST_SIGNAL_DEVIDER)

string(STRIP ${TEST_SIGNAL_DEVIDER} TEST_SIGNAL_DEVIDER)
math(EXPR TEST_SIGNAL_HZ10 "${REFERENCE_CLOCK_HZ} * 10 / (1 << ${TEST_SIGNAL_DEVIDER})")
floterise(TEST_SIGNAL_HZ ${TEST_SIGNAL_HZ10} 10)
message(STATUS "Actual test input signal freq: ${TEST_SIGNAL_HZ}Hz (/ ${TEST_SIGNAL_DEVIDER})")

#-------------------------------/Calculable-------------------------------------

set(MAIN_MEMORY_BASE        0x10000000) # memory base address

#---------------------------------bootloader------------------------------------
add_subdirectory(bootloader)
#--------------------------------/bootloader------------------------------------

#------------------------------------HDL----------------------------------------
include(Xilinx_build)
add_subdirectory(hdl)
#------------------------------------prj----------------------------------------

add_custom_command(OUTPUT ${PRJ_FILE_NAME} DEPENDS hdl)

#------------------------------------xst----------------------------------------

set(SYR_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.syr")
set(NGC_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.ngc")

make_xst(${SYR_FILE} ${NGC_FILE} ${PRJ_FILE_NAME} ${XST_FILE_NAME})

#-----------------------------------ngdbuild------------------------------------

set(NGD_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.ngd")
set(NGO_DIR	"${CMAKE_CURRENT_BINARY_DIR}/_ngo")

make_ngdbuild(${NGD_FILE} ${NGO_DIR} ${UCF_FILE_NAME} ${NGC_FILE})

#--------------------------------------map--------------------------------------

set(MAP_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}_map.ncd")
set(PCF_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.pcf")

make_map(${MAP_FILE} ${PCF_FILE} ${PART_NAME} ${NGD_FILE})

#--------------------------------------par--------------------------------------

set(NCD_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.ncd")

make_par(${NCD_FILE} ${MAP_FILE} ${PCF_FILE})

#-------------------------------------trce--------------------------------------

set(TWX_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.twx")
set(TWR_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.twr")

make_trce(${TWX_FILE} ${TWR_FILE} ${NCD_FILE} ${PCF_FILE})

#------------------------------------bitgen-------------------------------------

set(BIT_FILE	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.bit")

make_bitgen(${BIT_FILE} ${UT_FILE_NAME} ${NCD_FILE})

#------------------------------------/HDL---------------------------------------

#---------------------------------application-----------------------------------
add_subdirectory(application)
add_custom_command(OUTPUT ${APPLICATION_BINARY} DEPENDS application)
#--------------------------------/application-----------------------------------

#----------------------------------programm-------------------------------------

set(IMPACT_BATCH_FILE_CHIP_IN	${CMAKE_CURRENT_SOURCE_DIR}/programm_chip.cmd.in)
set(IMPACT_BATCH_FILE_FLASH_IN	${CMAKE_CURRENT_SOURCE_DIR}/programm_flash.cmd.in)

set(IMPACT_BATCH_FILE_CHIP	${CMAKE_CURRENT_BINARY_DIR}/programm_chip.cmd)
set(IMPACT_BATCH_FILE_FLASH	${CMAKE_CURRENT_BINARY_DIR}/programm_flash.cmd)

#set(MCS_CONFIG_IMAGE		${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.mcs)
set(MCS_FIRMWARE_IMAGE		${CMAKE_CURRENT_BINARY_DIR}/firmware.mcs)

set(IMPACT_LOG			${CMAKE_CURRENT_BINARY_DIR}/impact.log)

set(CDF_FILE			${CMAKE_CURRENT_BINARY_DIR}/cdffile.cdf)

configure_file(${IMPACT_BATCH_FILE_CHIP_IN} ${IMPACT_BATCH_FILE_CHIP} @ONLY)
configure_file(${IMPACT_BATCH_FILE_FLASH_IN} ${IMPACT_BATCH_FILE_FLASH} @ONLY)

build_mcs(${MCS_FIRMWARE_IMAGE}
    0x0 ${BIT_FILE}
    ${USER_CODE_FLASH_OFFSET} ${APPLICATION_BINARY}
    )
#append_data_to_file(${MCS_FIRMWARE_IMAGE} ${MCS_CONFIG_IMAGE}
#    ${APPLICATION_BINARY} ${USER_CODE_FLASH_OFFSET})

add_custom_target(flash_image DEPENDS ${MCS_FIRMWARE_IMAGE})

make_impact_programm(${IMPACT_BATCH_FILE_CHIP} ${BIT_FILE})
make_impact_flash(${IMPACT_BATCH_FILE_FLASH} ${MCS_FIRMWARE_IMAGE})

#---------------------------------/programm-------------------------------------
