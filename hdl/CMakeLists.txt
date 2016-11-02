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

set(TOP_MODULE_NAME	top)

add_subdirectory(scripts)

#-------------------------------------------------------------------------------
set(PRJ_FILE_NAME	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.prj")
set(XST_FILE_NAME	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.xst")
set(XST_IN_FILE_NAME	"${CMAKE_CURRENT_SOURCE_DIR}/${TOP_MODULE_NAME}.xst.in")
set(UT_FILE_NAME	"${CMAKE_CURRENT_BINARY_DIR}/${TOP_MODULE_NAME}.ut")
set(UT_IN_FILE_NAME	"${CMAKE_CURRENT_SOURCE_DIR}/${TOP_MODULE_NAME}.ut.in")

if (NOT PART_NAME)
    message(WARNING "PART_NAME not selected, default: xc6slx9-3-tqg144")
    set(PART_NAME "xc6slx9-3-tqg144")
    set(PART_NAME ${PART_NAME}	PARENT_SCOPE)
endif(NOT PART_NAME)

set(config_v_file_name_in	    "${CMAKE_CURRENT_SOURCE_DIR}/config.v.in")
set(config_v_file_name		    "${CMAKE_CURRENT_BINARY_DIR}/config.v")

#--------------------------------config-----------------------------------------

Verilog_GenControlMacro(UART0_ENABLED_STR "UART0_ENABLED" ${PERIPHERIAL_ENABLE_UART0})
Verilog_GenControlMacro(UART1_ENABLED_STR "UART1_ENABLED" ${PERIPHERIAL_ENABLE_UART1})
Verilog_GenControlMacro(TIMER_ENABLED_STR "TIMER_ENABLED" ${PERIPHERIAL_ENABLE_TIMER})
Verilog_GenControlMacro(ETHERNET_ENABLED_STR "ETHERNET_ENABLED" ${PERIPHERIAL_ENABLE_ETHERNET})
Verilog_GenControlMacro(MASTER_HYBRID_COUNTER_STR "MASTER_HYBRID_COUNTER" ${MASTER_HYBRID_COUNTER})

#---------------------------------files-----------------------------------------

add_subdirectory(ucf)
add_subdirectory(altor32)
add_subdirectory(soc)
add_subdirectory(memory)
add_subdirectory(mdio)
add_subdirectory(myminimac)
add_subdirectory(utils)

#-------------------------------top module--------------------------------------

file(GLOB
    TOP_MODULE_FILE
    "${TOP_MODULE_NAME}.v"
    )

build_prj(TOP_PRJ	    "verilog work	%f\n"	"${TOP_MODULE_FILE}")

add_custom_target(top_hdl_src
    SOURCES
	${TOP_MODULE_FILE}
    )

#-----------------------------------prj-----------------------------------------

configure_file(${config_v_file_name_in} ${config_v_file_name} @ONLY)

build_prj(ALTOR32_CORE_PRJ  "verilog altor32	%f\n"	"${ALTOR32_HDL_V}")
build_prj(SOC_V_PRJ	    "verilog soc	%f\n"	"${SOC_HDL_V}")
build_prj(SOC_VHDL_PRJ	    "vhdl    soc        %f\n"	"${SOC_HDL_VHDL}")
build_prj(MEMORY_PRJ	    "verilog memory	%f\n"	"${MEMORY_HDL_V}")
build_prj(MDIO_PRJ	    "verilog mdio	%f\n"	"${MDIO_HDL_V}")
build_prj(MYMINMAC_PRJ	    "verilog myminimac	%f\n"	"${MYMINMAC_HDL_V}")
build_prj(MYMINMAC_VHDL_PRJ "vhdl    myminimac	%f\n"	"${MYMINMAC_HDL_VHDL}")
build_prj(UTILS_PRJ	    "verilog utils	%f\n"	"${UTILS_HDL_V}")
build_prj(CONFIG_PRJ        "verilog config     %f\n"   "${config_v_file_name}")

set(PRJ_TEXT
    ${CONFIG_PRJ}
    ${ALTOR32_CORE_PRJ}
    ${SOC_V_PRJ}
    ${SOC_VHDL_PRJ}
    ${MDIO_PRJ}
    ${MYMINMAC_PRJ}
    ${MYMINMAC_VHDL_PRJ}
    ${UTILS_PRJ}
    ${TOP_PRJ}
    )

set(PRJ_BUILD ${PRJ_TEXT} ${MEMORY_PRJ})

file(WRITE ${PRJ_FILE_NAME}.tmp ${PRJ_BUILD})

add_custom_target(hdl
    COMMAND
	${SCRIPTS_DIR}/checksrc.sh ${PRJ_FILE_NAME}.tmp ${PRJ_FILE_NAME}
    DEPENDS
	memory_refresh
    )

#-----------------------------------xst-----------------------------------------

configure_file(${XST_IN_FILE_NAME} ${XST_FILE_NAME} @ONLY)

#-------------------------------------ut----------------------------------------

configure_file(${UT_IN_FILE_NAME} ${UT_FILE_NAME} @ONLY)

#-------------------------------------------------------------------------------

#-------------------------------------sym---------------------------------------
add_subdirectory(testbench)

set(prj_libs altor32 soc memory utils work mdio myminimac)
set(testlibs_list)
foreach(l ${prj_libs})
    list(APPEND testlibs_list -lib ${l})
endforeach()

set(test_include_path
    -i ${CMAKE_CURRENT_BINARY_DIR}
    -i ${CMAKE_CURRENT_SOURCE_DIR}
    -i ${UTILS_INCLUDE_PATH}
    -i ${MYMINIMAC_INCLUDE_PATH}
    )

build_prj(XILINX_VERILOG_glbl_PRJ   "verilog work	%f\n"	"${XILINX_VERILOG_glbl}")
build_prj(MEMORY_HDL_SIM_PRJ	    "verilog memory	%f\n"	"${MEMORY_HDL_SIM_V}")

foreach(testfile ${TESTBENCH_HDL_V})
    string(REGEX REPLACE ".*/(.*)\\.v" "\\1" testname ${testfile})
    message(STATUS "Creating testbench target for ${testname}")

    set(test_prj_file	    ${TESTBENCH_DIR}/${testname}.prj)
    set(test_executable	    ${TESTBENCH_DIR}/${testname}.exe)
    set(test_top_lvl_module "tb.${testname}")

    # create prj
    build_prj(TB_PRJ	"verilog tb	%f\n"	"${testfile}")

    file(WRITE ${test_prj_file}.tmp
        ${PRJ_TEXT}
        ${MEMORY_HDL_SIM_PRJ}
	${TB_PRJ}
	${XILINX_VERILOG_glbl_PRJ}
	)

    add_custom_target(${testname}.prj
	COMMAND
	    ${SCRIPTS_DIR}/checksrc.sh ${test_prj_file}.tmp ${test_prj_file}
	DEPENDS
	    memory_refresh_sim
        COMMENT
            "Refreshing ${testname}.prj"
	)

    make_Behavioral_testbench(
	"${testlibs_list}"
	${test_executable}
	${test_prj_file}
	${test_top_lvl_module}
	${TESTBENCH_DIR}
	"${test_include_path}"
	)
    add_dependencies(${PROJECT_NAME}_fuse.${test_top_lvl_module} ${testname}.prj)

endforeach()

#------------------------------------/sym---------------------------------------

math(EXPR _MEMORY_UNIT_SIZE "${MEMORY_UNIT_SIZE}")

set(PRJ_FILE_NAME           ${PRJ_FILE_NAME}            PARENT_SCOPE)
set(TOP_MODULE_NAME         ${TOP_MODULE_NAME}          PARENT_SCOPE)
set(PRJ_TEXT                ${PRJ_TEXT}                 PARENT_SCOPE)
set(XST_FILE_NAME           ${XST_FILE_NAME}            PARENT_SCOPE)
set(UT_FILE_NAME            ${UT_FILE_NAME}		PARENT_SCOPE)
set(UCF_FILE_NAME           ${UCF_FILE_NAME}            PARENT_SCOPE)
set(F_INPUTS_COUNT          ${F_INPUTS_COUNT}           PARENT_SCOPE)

set(USER_CODE_FLASH_OFFSET  ${USER_CODE_FLASH_OFFSET}   PARENT_SCOPE)

set(MYMINMAC_RX_SLOTS       ${MYMINMAC_RX_SLOTS}        PARENT_SCOPE)
set(MYMINMAC_TX_SLOTS       ${MYMINMAC_TX_SLOTS}        PARENT_SCOPE)
set(MAC_TX_MEM_BASE         ${MAC_TX_MEM_BASE}          PARENT_SCOPE)
set(MAC_RX_MEM_BASE         ${MAC_RX_MEM_BASE}          PARENT_SCOPE)
set(MAC_CTL_BASE            ${MAC_CTL_BASE}             PARENT_SCOPE)
set(MTU                     ${MTU}                      PARENT_SCOPE)
set(MEMORY_UNIT_SIZE        ${_MEMORY_UNIT_SIZE}        PARENT_SCOPE)
set(FREQMETERS_BASE         ${FREQMETERS_BASE}          PARENT_SCOPE)

set(TESTBENCH_TOP           ${TESTBENCH_TOP}            PARENT_SCOPE)
set(TESTBENCH_INCLUDE       ${test_include_path}        PARENT_SCOPE)
