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

cmake_minimum_required(VERSION 3.0.2)

#-------------------------------------ucf---------------------------------------

set(UCF_FILE_NAME_IN    "${CMAKE_CURRENT_SOURCE_DIR}/xilinx-${PART_NAME}.ucf.in")
set(UCF_FILE_NAME       "${CMAKE_CURRENT_BINARY_DIR}/xilinx-${PART_NAME}.ucf")

configure_file(${UCF_FILE_NAME_IN} ${UCF_FILE_NAME} @ONLY)

file(STRINGS ${UCF_FILE_NAME_IN} F_INPUTS_COUNT
    REGEX "^NET.*Fin\\[.*\\].*LOC = "
    )
list(LENGTH F_INPUTS_COUNT F_INPUTS_COUNT)

file(STRINGS ${UCF_FILE_NAME_IN} LEDS_COUNT
    REGEX "^NET.*leds_io\\[.*\\].*LOC = "
    )
list(LENGTH LEDS_COUNT LEDS_COUNT)

#-------------------------------------ucp---------------------------------------

set(ucp_filename    "${CMAKE_CURRENT_SOURCE_DIR}/xilinx-${PART_NAME}.ucp")
file(STRINGS ${ucp_filename} USER_CODE_FLASH_OFFSET
    REGEX "USER_CODE_FLASH_OFFSET=.*"
    )
string(REGEX REPLACE "USER_CODE_FLASH_OFFSET=(.*)" "\\1" USER_CODE_FLASH_OFFSET ${USER_CODE_FLASH_OFFSET})
message(STATUS "User code will be at ${USER_CODE_FLASH_OFFSET} in flash")

file(STRINGS ${ucp_filename} USE_PHISICAL_INPUTS
    REGEX "USE_PHISICAL_INPUTS=.*"
    )
string(REGEX REPLACE "USE_PHISICAL_INPUTS=(.*)" "\\1" USE_PHISICAL_INPUTS ${USE_PHISICAL_INPUTS})
if (USE_PHISICAL_INPUTS EQUAL 0)
    set(F_INPUTS_COUNT  24)
    set(USE_PHISICAL_INPUTS "// Macro USE_PHISICAL_INPUTS is not defined")
    message(STATUS "Using test signals as inputs: 24")
else()
    set(USE_PHISICAL_INPUTS "`define     USE_PHISICAL_INPUTS")
    message(STATUS "Using physical inputs: ${F_INPUTS_COUNT}")
endif()

#-------------------------------------------------------------------------------

add_custom_target(constraints_src
    SOURCES
        ${UCF_FILE_NAME_IN}
        ${ucp_filename}
    )

set(UCF_FILE_NAME           ${UCF_FILE_NAME}            PARENT_SCOPE)
set(USER_CODE_FLASH_OFFSET  ${USER_CODE_FLASH_OFFSET}   PARENT_SCOPE)
set(F_INPUTS_COUNT          ${F_INPUTS_COUNT}           PARENT_SCOPE)
set(LEDS_COUNT              ${LEDS_COUNT}               PARENT_SCOPE)
set(USE_PHISICAL_INPUTS     ${USE_PHISICAL_INPUTS}      PARENT_SCOPE)
