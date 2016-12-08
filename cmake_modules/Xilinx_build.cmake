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

function(build_prj resultvar pattern filelist)
    set(result)
    foreach(f ${filelist})
        string(REPLACE "%f" ${f} line ${pattern})
        list(APPEND result ${line})
    endforeach(f)
    set(${resultvar} ${result} PARENT_SCOPE)
endfunction(build_prj)

function(make_prj PRJ_FILE_NAME PRJ_TEXT)
    file(WRITE ${PRJ_FILE_NAME} ${PRJ_TEXT})
    add_custom_command(
	OUTPUT
	    ${PRJ_FILE_NAME}
	COMMAND
	    $(CMAKE_COMMAND) -E touch_nocreate ${PRJ_FILE_NAME}
#	DEPENDS
#	    altor32_sources soc_sources top_hdl_sources memory_sources
	COMMENT
	    "Building PRJ file"
	)
    add_custom_target(${PROJECT_NAME}_prj DEPENDS ${PRJ_FILE_NAME})
endfunction(make_prj)

function(make_xst SYR_FILE NGC_FILE PRJ_FILE_NAME XST_FILE_NAME)
    add_custom_command(
	OUTPUT
	    ${SYR_FILE}
	    ${NGC_FILE}
	COMMAND
	    ${XILINX_xst} -ifn ${XST_FILE_NAME} -ofn ${SYR_FILE}
	DEPENDS
	    ${PRJ_FILE_NAME} ${XST_FILE_NAME}
	COMMENT
	    "Compile HDL files"
	)

    add_custom_target(${PROJECT_NAME}_xst ALL DEPENDS ${SYR_FILE}) #
endfunction(make_xst)

function(make_ngdbuild PART_NAME NGD_FILE NGO_DIR UCF_FILE_NAME NGC_FILE)
    add_custom_command(OUTPUT ${NGD_FILE}
	COMMAND
	    ${XILINX_ngdbuild} -dd ${NGO_DIR}
		-nt timestamp
		-uc ${UCF_FILE_NAME}
		-p ${PART_NAME}
                -intstyle ise
                -verbose
		${NGC_FILE} ${NGD_FILE}
	DEPENDS
	    ${NGC_FILE} ${UCF_FILE_NAME}
	COMMENT
	    "Starting ngdbuild"
	)

    add_custom_target(${PROJECT_NAME}_ngdbuild DEPENDS ${NGD_FILE})
endfunction(make_ngdbuild)

function(make_map MAP_FILE PCF_FILE PART_NAME NGD_FILE)
    add_custom_command(
	OUTPUT
	    ${MAP_FILE}
	    ${PCF_FILE}
	COMMAND
	    ${XILINX_map} -p ${PART_NAME}
		-w -logic_opt off
		-ol high -t 1 -xt 0
		-register_duplication off
		-global_opt off
		-mt off -ir off
                -pr off
                -lc auto
		-power off
		-o ${MAP_FILE}
		${NGD_FILE}
		${PCF_FILE}
	DEPENDS
	    ${NGD_FILE}
	COMMENT
	    "Maping"
	)

    add_custom_target(${PROJECT_NAME}_map
	DEPENDS
	    ${MAP_FILE}
	    ${PCF_FILE}
	)
endfunction(make_map)

function(make_par NCD_FILE MAP_FILE PCF_FILE)
    add_custom_command(
	OUTPUT
	    ${NCD_FILE}
	COMMAND
	    ${XILINX_par}
		-ol high
		-mt off
		${MAP_FILE}
		-w ${NCD_FILE}
		${PCF_FILE}
	DEPENDS
	    ${MAP_FILE}
	    ${PCF_FILE}
	COMMENT
	    "Paring"
	)

    add_custom_target(${PROJECT_NAME}_par DEPENDS ${NCD_FILE})
endfunction(make_par)

function(make_trce TWX_FILE TWR_FILE NCD_FILE PCF_FILE)
    add_custom_command(
	OUTPUT
	    ${TWX_FILE}
	    ${TWR_FILE}
	COMMAND
	    ${XILINX_trce} -v 3
		-s 2
		-n 3
		-fastpaths
		-xml ${TWX_FILE}
		${NCD_FILE}
		-o ${TWR_FILE}
		${PCF_FILE}
	DEPENDS
	    ${PCF_FILE} ${NCD_FILE}
	COMMENT
	    "Running trce"
	)

    add_custom_target(${PROJECT_NAME}_trace
	DEPENDS
	    ${TWX_FILE}
	    ${TWR_FILE}
	)
endfunction(make_trce)

function(make_bitgen BIT_FILE UT_FILE_NAME NCD_FILE)
    add_custom_command(
	OUTPUT
	    ${BIT_FILE}
	COMMAND
	    ${XILINX_bitgen}
		-f ${UT_FILE_NAME}
		-m # generate .msk
		${NCD_FILE}
		${BIT_FILE}
	DEPENDS
	    ${UT_FILE_NAME}
	    ${NCD_FILE}
	COMMENT
	    "Generating bitstream"
	)

    add_custom_target(firmware DEPENDS ${BIT_FILE})
endfunction(make_bitgen)

function(make_impact_programm CMD_FILE BIT_FILE)
    add_custom_target(programm
	COMMAND
	    ${XILINX_impact}
		-batch ${CMD_FILE}
	DEPENDS
	    ${CMD_FILE}
	    ${BIT_FILE}
	COMMENT
	    "Programming target..."
	)
endfunction(make_impact_programm)

function(make_impact_flash CMD_FILE FLASH_IMAGE)
    add_custom_target(flash
	COMMAND
	    ${XILINX_impact}
		-batch ${CMD_FILE}
	DEPENDS
	    ${CMD_FILE}
	    ${FLASH_IMAGE}
	COMMENT
	    "Programming flash..."
	)
endfunction(make_impact_flash)

function(make_Behavioral_testbench LIBS BENCH_EXECUTABLE TB_PRJ TOP_LVL_MODULE TESTBENCH_DIR INCLUDE_PATH)
    add_custom_command(
	OUTPUT
	    ${BENCH_EXECUTABLE}
	COMMAND
	    ${XILINX_fuse}
		${INCLUDE_PATH}
		-intstyle ise
		-incremental
		-lib unisims_ver
		-lib unimacro_ver
		-lib xilinxcorelib_ver
		-lib secureip
		${LIBS}
		-o ${BENCH_EXECUTABLE}
		-prj ${TB_PRJ}
		${TOP_LVL_MODULE}
		work.glbl
	DEPENDS
	    ${TB_PRJ}
	WORKING_DIRECTORY
	    ${TESTBENCH_DIR}
	COMMENT
            "Making testbench ${TOP_LVL_MODULE}"
	)

    add_custom_target(${PROJECT_NAME}_fuse.${TOP_LVL_MODULE}
	DEPENDS ${BENCH_EXECUTABLE}
	)
    add_custom_target(${TOP_LVL_MODULE}.run
	DEPENDS ${PROJECT_NAME}_fuse.${TOP_LVL_MODULE}
	COMMAND
	    export PATH=$PATH:${XILINX_DIR} && export XILINX=${XILINX_DIR}/../.. && ${BENCH_EXECUTABLE} -gui
	WORKING_DIRECTORY
	    ${TESTBENCH_DIR}
	)
endfunction(make_Behavioral_testbench)

function(build_mcs MCS_FLAH_IMAGE offset0 file0)
    set(argsList -u ${offset0} ${file0})
    set(files ${file0})
    if (${ARGC} GREATER 3)
        SET(ARGS    ${ARGV})
        list(REMOVE_AT ARGS 0 1 2)
        list(LENGTH ARGS args_length)
        math(EXPR args_length "${args_length} - 1")
        foreach(i RANGE 0 ${args_length} 2)
            list(GET ARGS ${i} offset)
            math(EXPR i1 "${i} + 1")
            list(GET ARGS ${i1} file)
            list(APPEND argsList -data_file up ${offset} ${file})
            list(APPEND files ${file})
        endforeach()
    endif()
    set(product "${argsList}")
    add_custom_command(
        OUTPUT
            ${MCS_FLAH_IMAGE}
        COMMAND
            ${XILINX_promgen} -w -spi -c 0xff -p mcs -o ${MCS_FLAH_IMAGE} ${product}
        DEPENDS
            "${files}"
        )
endfunction(build_mcs)

function(append_data_to_file RESULT iHEX_FILE BINARY_FILE OFFSET)
    set(iHEX_FILE2BIN   ${iHEX_FILE}.bin)
    set(RESULT2BIN      ${RESULT}.bin)
    add_custom_command(
        OUTPUT ${RESULT}
        DEPENDS
            ${iHEX_FILE} ${BINARY_FILE}
        COMMAND objcopy -Iihex -Obinary ${iHEX_FILE} ${iHEX_FILE2BIN}
        COMMAND ${tools_DIR}/merge_binary.py --basefile=${iHEX_FILE2BIN}
            --importfile=${BINARY_FILE} --offset=${OFFSET} > ${RESULT2BIN}
        COMMAND objcopy -Ibinary -Oihex ${RESULT2BIN} ${RESULT}
        COMMENT
            "Appending file ${BINARY_FILE} to ${iHEX_FILE}"
    )
endfunction()

function(Verilog_GenControlMacro OUTVALUE MACRO STATE)
    if (${STATE})
        set(result "`define ${MACRO}")
    else()
        set(result "// Macro ${MACRO} is not defined")
    endif()
    set(${OUTVALUE} ${result} PARENT_SCOPE)
endfunction()

function(make_netgen NCD_FILE PCF_FILE OUTDIR OUT_VERILOG OUT_SDF)
    get_filename_component(module_name ${NCD_FILE} NAME_WE)
    set(verilog_sim_module  ${OUTDIR}/${module_name}.v)
    set(sdf_sim_module      ${OUTDIR}/${module_name}.sdf)
    set(${OUT_VERILOG}      ${verilog_sim_module}       PARENT_SCOPE)
    set(${OUT_SDF}          ${sdf_sim_module}           PARENT_SCOPE)
    add_custom_command(
        OUTPUT
            ${verilog_sim_module} ${sdf_sim_module}
        COMMAND
            mkdir -p ${OUTDIR}
        COMMAND
            ${XILINX_netgen} -sim
                -ofmt verilog
                -intstyle ise
                -dir ${OUTDIR}
                -pcf ${PCF_FILE}
                -w
                ${NCD_FILE}
        DEPENDS
            ${NGC_FILE} ${PCF_FILE}
        COMMENT
            "Generating NetGen Functional Simulation"
        )
    add_custom_target(mk_ps_netgen
        DEPENDS
            ${verilog_sim_module}
            ${sdf_sim_module}
        )
endfunction(make_netgen)

function(make_Timing_testbench BENCH_EXECUTABLE TB_PRJ TESTBENCH_DIR TOP_LVL_MODULE SDF INCLUDE_PATH)
    add_custom_command(
        OUTPUT
            ${BENCH_EXECUTABLE}
        COMMAND
            ${XILINX_fuse}
                ${INCLUDE_PATH}
                -intstyle ise
                -incremental
                -lib unisims_ver
                -lib unimacro_ver
                -lib xilinxcorelib_ver
                -lib simprims_ver
                -lib secureip
                -o ${BENCH_EXECUTABLE}
                -prj ${TB_PRJ}
                ${TOP_LVL_MODULE}
                work.glbl
        DEPENDS
            ${TB_PRJ}
        WORKING_DIRECTORY
            ${TESTBENCH_DIR}
        COMMENT
            "Making timing testbench"
        )

    add_custom_target(tb.top_timing
        DEPENDS ${BENCH_EXECUTABLE}
        )
    add_custom_target(tb.top_timing.run
        DEPENDS tb.top_timing
        COMMAND
            export PATH=$PATH:${XILINX_DIR} && export XILINX=${XILINX_DIR}/../.. && ${BENCH_EXECUTABLE} -gui -sdftyp ${SDF}
        WORKING_DIRECTORY
            ${TESTBENCH_DIR}
        )
endfunction(make_Timing_testbench)
