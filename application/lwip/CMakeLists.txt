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

############################## fill variables ##################################

set(LWIPDIR                     ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(VARS_MAKEHALPER             ${CMAKE_CURRENT_SOURCE_DIR}/makehalper.mk)
set(Filelists_mk_file           ${LWIPDIR}/Filelists.mk)

function(extract_lwip_variable_value OUTVAR VARIABLE_NAME)
    execute_process(
        COMMAND
            ${CMAKE_MAKE_PROGRAM} -f ${VARS_MAKEHALPER}
            print-${VARIABLE_NAME} LWIPDIR=${LWIPDIR} FILELIST_MK=${Filelists_mk_file}
        OUTPUT_VARIABLE res
        )
    string(REPLACE " " ";" res ${res}) # make a list
    set(${OUTVAR}   ${res}  PARENT_SCOPE)
endfunction(extract_lwip_variable_value)


set(LWIP_VARIABLES
        COREFILES
        CORE4FILES
        CORE6FILES
        APIFILES
        NETIFFILES
        SIXLOWPAN
        PPPFILES
        SNMPFILES
        HTTPDFILES
        LWIPERFFILES
        SNTPFILES
        NETBIOSNSFILES
        LWIPAPPFILES
        )

foreach(lwip_var ${LWIP_VARIABLES})
    extract_lwip_variable_value(LWIP_VAR_${lwip_var} ${lwip_var})
endforeach()

################################## DIFFENITIONS ################################


################################## BUILD TARGETS ###############################



################################################################################

add_custom_target(LWIP_BUILD_HALPERS
    SOURCES
        ${VARS_MAKEHALPER}
    )
