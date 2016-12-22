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

find_path (
     XILINX_DIR
     names
	xst
	ngdbuild
	map
	par
	trce
	bitgen
     HINTS
	/opt
	/mnt
	/mnt/Steam/Xilinx/14.3/ISE_DS/ISE/bin/lin64
	/mnt/ssd_build/Xilinx/14.3/ISE_DS/ISE/bin/lin64
	/media/truecrypt1/Xilinx/14.3/ISE_DS/ISE/bin/lin64
     )

if(XILINX_DIR)
    set(XILINX_FOUND TRUE)
    message(STATUS "Xilinx found: ${XILINX_DIR}")

    set(utils	xst ngdbuild map par trce bitgen fuse impact promgen ngcbuild netgen)
    foreach(u ${utils})
	find_file(
	    XILINX_${u}
	    ${u}
	    NO_DEFAULT_PATH
	    PATHS
		${XILINX_DIR}
	    )
    endforeach(u)

    # $XILINX/verilog/src/glbl.v
    find_file(
	XILINX_VERILOG_glbl
	glbl.v
	PATHS
	    ${XILINX_DIR}/../../verilog/src
	)

else (XILINX_DIR)
    set(XILINX_FOUND FALSE)
    message(FATAL_ERROR "Xilinx not found. Please install it")
endif(XILINX_DIR)

mark_as_advanced(XILINX_DIR)

