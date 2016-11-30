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

# avoid CMAKE to test compiler
SET(CMAKE_CROSSCOMPILING TRUE)
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_C_COMPILER_WORKS TRUE)

find_package(PythonInterp 3 REQUIRED)

add_subdirectory(tools)

# указать путь до модулей
set(CMAKE_MODULE_PATH
	${CMAKE_MODULE_PATH}
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include("halpers")

find_package(Xilinx REQUIRED)

#-----------------------------------config--------------------------------------

set(PART_FAMILY             "SPARTAN6"
    CACHE STRING "Xilinx FPGA family")
set(PART_NAME               "xc6slx9-3-tqg144"
    CACHE STRING "Xilinx FPGA model to synthesis")
set(BOARD_NAME               "ZR-Tech_v2.00"
    CACHE STRING "Board name")
set(INPUT_CLOCK_HZ          48000000
    CACHE STRING "FPGA main clock source (Hz)")
set(SPI_FLASH_CHIP          "W25Q32BV"
    CACHE STRING "SPI flash memory part name")
set(FPGA_DP_MEMORY_USE      28
    CACHE STRING "Number of RAMB16BWERs to use as main memory")

set(HEADER_W1               0x03030303
    CACHE STRING "Application syncronisation word 1")
set(HEADER_W2               0x1f1f1f1f
    CACHE STRING "Application syncronisation word 2")

set(DEBUGER_AT_BOOTLOADER   OFF
    CACHE BOOL   "Start debuger in bootloader")

#--- peripherial ----
set(PERIPHERIAL_ENABLE_TIMER ON
    CACHE BOOL  "Enable timer at 0x12000100")
set(PERIPHERIAL_ENABLE_UART0 ON
    CACHE BOOL  "Enable UART0 IPCore at 0x12000000")
set(PERIPHERIAL_ENABLE_UART1 OFF
    CACHE BOOL  "Enable UART1 IPCore at 0x12000700")
set(PERIPHERIAL_ENABLE_GPIO  ON
    CACHE BOOL  "Enable i2c IPCore at 0x12000400")
set(PERIPHERIAL_ENABLE_I2C   OFF
    CACHE BOOL  "Enable i2c IPCore at 0x12000600")
set(PERIPHERIAL_ENABLE_ETHERNET ON
    CACHE BOOL  "Enable ETHERNET IPCore at 0x11000000")
#--- /peripherial ---

#--- SERIALs ----
set(UART0_BAUD              115200
    CACHE STRING "UART0 (debug) speed")
set(UART1_BAUD              921600
    CACHE STRING "UART1 speed")
set(MDIO_BAUD               2500000
    CACHE STRING "MDIO speed")
set(SPI_CLK_DEVIDER_LEN     4
    CACHE STRING "Boot SPI baud counter length (bits)")
set(I2C_BAUD                100000
    CACHE STRING "i2c speed")
#--- /SERIALs ---

set(MASTER_FREQ_COUNTER_LEN  28
    CACHE STRING "Master clock counter length (in bits)")
set(MASTER_HYBRID_COUNTER   ON
    CACHE BOOL "Use hybrid binary+gray counter for Master clock")
set(INPUT_FREQ_COUNTER_LEN  20
    CACHE STRING "Input clock counter length (in bits)")
set(TEST_SIGNAL_EVERY_MS    1000
    CACHE STRING "Test input signal present aproximaly every")

set(FREF_PLL_MULTIPLYER     20
    CACHE STRING "Reference clock multiplier")

set(FREF_CLOCK_DEVIDER      10
    CACHE STRING "Reference clock devider")

set(CPU_PLL_MULTIPLYER      25
    CACHE STRING "Input clock to CPU multiplier")

set(CPU_CLOCK_DEVIDER       24
    CACHE STRING "Input clock to CPU devider")

set(SYSTEM_HEAP_SIZE        4096
    CACHE STRING "System memory heap size")

#---------------------------------------Servers---------------------------------

set(SERVER_UDP              ON
    CACHE BOOL   "Enable UDP server")
set(SERVER_UDP_PORT         9128
    CACHE STRING "Enable UDP server port number")
set(SERVER_WEBSOC           OFF
    CACHE BOOL   "Enable WebSocket server")
set(SERVER_WEBSOC_PORT      3897
    CACHE STRING "Enable WebSocket server port number")
set(SERVER_HTTP             OFF
    CACHE BOOL   "Enable HTTP server")

#-------------------------------------------------------------------------------

set(ETHERNET_MAC_ADDRESS_MSB  0c
    CACHE STRING  "First byte of ethernet MAC address (execute make gen_mac to renew)")

set(ETHERNET_MAC_ADDRESS_FORCE  NO
    CACHE BOOL  "Force test MAC address")

set(ETHERNET_MAC_ADDRESS_FORCED  0c:98:77:c3:12:a3
    CACHE STRING  "Forced test MAC address")

set(ETHERNET_STATIC_IP_ADDR "192.168.0.199"
    CACHE STRING  "Default static IP addr")

set(ETHERNET_STATIC_IP_NETMASK "255.255.255.0"
    CACHE STRING  "Default static IP netmask")

set(ETHERNET_STATIC_IP_GATEWAY "192.168.0.1"
    CACHE STRING  "Default static IP gateway")

set(ETHERNET_USE_DHCP       YES
    CACHE BOOL   "Try to obtain ip address trouth DHCP on startup")

set(ETHERNET_SKIP_UDP_CHECKSUMS YES
    CACHE BOOL   "Disable UDP checksums calculation and check")

#-------------------------------------------------------------------------------

set(SITE_FILENAME_MAX       21
    CACHE STRING "Site path+filename max size")

set(SITE_FILEEXT_MAX        3
    CACHE STRING "Site filenamepath max size")

set(SITE_STATIC_CACHE       128
    CACHE STRING "SPI flash read cache")


#--- test firmware ---
set(SIM_TEST_FREQMETER      OFF
    CACHE BOOL  "Bootloader firmware tests freqmeter IP-core")
set(SIM_TEST_MULTIPLICATION OFF
    CACHE BOOL  "Bootloader firmware tests processor multiplication")
set(SIM_TEST_MINIMAC        OFF
    CACHE BOOL  "Bootloader firmware tests myminimac IP-core")
set(SIM_TEST_MINIMAC_SLOT_LOGICK OFF
    CACHE BOOL  "Bootloader firmware tests myminimac IP-core rx logick")
set(SIM_TEST_MDIO           OFF
    CACHE BOOL  "Bootloader firmware tests myminimac IP-core MDIO interface")
set(SIM_TEST_I2C            OFF
    CACHE BOOL  "Bootloader firmware tests I2C interface")
set(SIM_TEST_GPIO           OFF
    CACHE BOOL  "Bootloader firmware tests GPIO pins")

#----------------------------------/config--------------------------------------

#--------------------------------Calculable-------------------------------------

math(EXPR RES_PLL_CLOCK_HZ
    "${INPUT_CLOCK_HZ} * ${FREF_PLL_MULTIPLYER}")

math(EXPR REFERENCE_CLOCK_HZ
    "${RES_PLL_CLOCK_HZ} / ${FREF_CLOCK_DEVIDER}")

math(EXPR CPU_CLOCK_HZ
    "${INPUT_CLOCK_HZ} * ${CPU_PLL_MULTIPLYER} / ${CPU_CLOCK_DEVIDER}")

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
            --maxdev=${MASTER_FREQ_COUNTER_LEN}
            --freq=${REFERENCE_CLOCK_HZ}
            --target=${TEST_SIGNAL_EVERY_MS}
    OUTPUT_VARIABLE TEST_SIGNAL_DEVIDER)

string(STRIP ${TEST_SIGNAL_DEVIDER} TEST_SIGNAL_DEVIDER)
math(EXPR TEST_SIGNAL_HZ10 "${REFERENCE_CLOCK_HZ} * 10 / (1 << ${TEST_SIGNAL_DEVIDER})")
floterise(TEST_SIGNAL_HZ ${TEST_SIGNAL_HZ10} 10)
message(STATUS "Actual test input signal freq: ${TEST_SIGNAL_HZ}Hz (/ ${TEST_SIGNAL_DEVIDER})")

# check if RMII clock == 50 MHz
math(EXPR CPU_CLOCK_HZ
    "${INPUT_CLOCK_HZ} * ${CPU_PLL_MULTIPLYER} / ${CPU_CLOCK_DEVIDER}")
message(STATUS "CPU clock is ${CPU_CLOCK_HZ}")

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

#-------------------------------------WWW---------------------------------------
if(SERVER_HTTP)
    add_subdirectory(www)
    add_custom_command(OUTPUT ${SITE_BLOB_NAME} DEPENDS site2blob)
endif()
#------------------------------------/WWW---------------------------------------

#---------------------------------application-----------------------------------
# position of site blob need for application
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "print('0x{:X}'.format(${USER_CODE_FLASH_OFFSET} + ${MEMORY_UNIT_SIZE_BYTES} * ${FPGA_DP_MEMORY_USE}))"
    OUTPUT_VARIABLE SITE_BLOB_START
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
message(STATUS "Site blob will be writen in flash at ${SITE_BLOB_START}")

add_subdirectory(application)
add_custom_command(OUTPUT ${APPLICATION_BINARY} DEPENDS application)
#--------------------------------/application-----------------------------------

#-----------------------------post-syntesis sim---------------------------------

set(PS_NETLIST_DIR  ${CMAKE_CURRENT_BINARY_DIR}/ps_netlist)

make_netgen(${NCD_FILE} ${PCF_FILE} ${PS_NETLIST_DIR} VERILOG_NETLIST SDF_FILE)
add_dependencies(mk_ps_netgen ${PROJECT_NAME}_par)

set(PS_TEST_EXECUTABLE ${PS_NETLIST_DIR}/${VERILOG_NETLIST}.exe)
set(PS_TEST_PRJ_FILE   ${PS_NETLIST_DIR}/${VERILOG_NETLIST}.prj)

build_prj(PS_TEST_TOP_PRJ           "verilog work	%f\n"   "${VERILOG_NETLIST}")
build_prj(PS_TB_TOP                 "verilog work       %f\n"   "${TESTBENCH_TOP}")
build_prj(XILINX_VERILOG_glbl_PRJ   "verilog work	%f\n"	"${XILINX_VERILOG_glbl}")

file(WRITE ${PS_TEST_PRJ_FILE}
    ${PS_TEST_TOP_PRJ}
    ${PS_TB_TOP}
    ${XILINX_VERILOG_glbl_PRJ}
    )

make_Timing_testbench(${PS_TEST_EXECUTABLE}
    ${PS_TEST_PRJ_FILE}
    ${PS_NETLIST_DIR}
    tb_top
    ${SDF_FILE}
    "${TESTBENCH_INCLUDE}"
    )
add_dependencies(tb.top_timing mk_ps_netgen)

#----------------------------/post-syntesis sim---------------------------------

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

if(SERVER_HTTP)
    build_mcs(${MCS_FIRMWARE_IMAGE}
        0x0 ${BIT_FILE}
        ${USER_CODE_FLASH_OFFSET} ${APPLICATION_BINARY}
        ${SITE_BLOB_START}  ${SITE_BLOB_NAME}
        )
else()
    build_mcs(${MCS_FIRMWARE_IMAGE}
        0x0 ${BIT_FILE}
        ${USER_CODE_FLASH_OFFSET} ${APPLICATION_BINARY}
        )
endif()

add_custom_target(flash_image DEPENDS ${MCS_FIRMWARE_IMAGE})

make_impact_programm(${IMPACT_BATCH_FILE_CHIP} ${BIT_FILE})
make_impact_flash(${IMPACT_BATCH_FILE_FLASH} ${MCS_FIRMWARE_IMAGE})

#---------------------------------/programm-------------------------------------

#------------------------------------tests--------------------------------------

add_subdirectory(tests)

#-----------------------------------/tests--------------------------------------

