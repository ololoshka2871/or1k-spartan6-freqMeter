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

#-------------------------------------ucf---------------------------------------

set(UCF_FILE_NAME_IN    "${CMAKE_CURRENT_SOURCE_DIR}/xilinx-${DEVICE_CHIP_NAME}-${DEVICE_BOARD_NAME}.ucf.in")
set(UCF_FILE_NAME       "${CMAKE_CURRENT_BINARY_DIR}/xilinx-${DEVICE_CHIP_NAME}-${DEVICE_BOARD_NAME}.ucf")

if(NOT PERIPHERIAL_ENABLE_GPIO)
    set(GPIO_DISABLE "#")
endif()

if((NOT PERIPHERIAL_ENABLE_I2C) AND (NOT PERIPHERIAL_ENABLE_GPIO))
    set(I2C_DISABLE "#")
endif()

if(NOT PERIPHERIAL_ENABLE_UART0)
    set(UART0_DISABLE "#")
endif()

configure_file(${UCF_FILE_NAME_IN} ${UCF_FILE_NAME} @ONLY)

file(STRINGS ${UCF_FILE_NAME_IN} F_INPUTS_COUNT
    REGEX "^NET.*Fin\\[.*\\].*LOC = "
    )
list(LENGTH F_INPUTS_COUNT F_INPUTS_COUNT)

file(STRINGS ${UCF_FILE_NAME} GPIO_COUNT
    REGEX "^NET.*gpio\\[.*\\].*LOC = "
    )
list(LENGTH GPIO_COUNT GPIO_COUNT)
if (GPIO_COUNT EQUAL 0)
    set(GPIO_PRESENT        0)
else()
    set(GPIO_PRESENT        1)
endif()

if ((NOT PERIPHERIAL_ENABLE_I2C) AND PERIPHERIAL_ENABLE_GPIO)
    file(STRINGS ${UCF_FILE_NAME} I2C_COUNT
        REGEX "^NET.*i2c_.*LOC = "
        )
    list(LENGTH I2C_COUNT I2C_COUNT)
    math(EXPR GPIO_COUNT "${GPIO_COUNT} + ${I2C_COUNT}")
endif()

if (PERIPHERIAL_ENABLE_GPIO)
    message(STATUS "GPIO count: ${GPIO_COUNT}")
    if (NOT PERIPHERIAL_ENABLE_I2C)
        math(EXPR I2C_OVER_GPIO_SDA_PIN "${GPIO_COUNT} - 2")
        math(EXPR I2C_OVER_GPIO_SCL_PIN "${GPIO_COUNT} - 1")

        message(STATUS "I2C over GPIO (bitbang) SDA => GPIO[${I2C_OVER_GPIO_SDA_PIN}]\tSCL => GPIO[${I2C_OVER_GPIO_SCL_PIN}]")
    endif()
else()
    message(STATUS "GPIO disabled")
endif()

#-------------------------------------ucp---------------------------------------

set(ucp_filename    ${UCF_FILE_NAME_IN})
file(STRINGS ${ucp_filename} USER_CODE_FLASH_OFFSET
    REGEX "USER_CODE_FLASH_OFFSET=.*"
    )
string(REGEX REPLACE ".*USER_CODE_FLASH_OFFSET=(.*)" "\\1" USER_CODE_FLASH_OFFSET ${USER_CODE_FLASH_OFFSET})
message(STATUS "User code will be at ${USER_CODE_FLASH_OFFSET} in flash")

file(STRINGS ${ucp_filename} USE_PHISICAL_INPUTS
    REGEX "USE_PHISICAL_INPUTS=.*"
    )
string(REGEX REPLACE ".*USE_PHISICAL_INPUTS=(.*)" "\\1" USE_PHISICAL_INPUTS ${USE_PHISICAL_INPUTS})
if (USE_PHISICAL_INPUTS EQUAL 0 OR F_INPUTS_COUNT EQUAL 0)
    set(F_INPUTS_COUNT  24)
    set(USE_PHISICAL_INPUTS "// Macro USE_PHISICAL_INPUTS is not defined")
    message(STATUS "Using test signals as inputs: 24")
    execute_process(
        COMMAND
            sed -i "s|^\\(NET.*Fin\\[.*\\].*LOC = \\)|# \\1|" ${UCF_FILE_NAME}
        )
else()
    set(USE_PHISICAL_INPUTS "`define     USE_PHISICAL_INPUTS")
    message(STATUS "Using physical inputs: ${F_INPUTS_COUNT}")
endif()

file(STRINGS ${ucp_filename} MEMORY_UNIT_SIZE
    REGEX "MEMORY_UNIT_SIZE=.*"
    )
string(REGEX REPLACE ".*MEMORY_UNIT_SIZE=(.*)" "\\1" MEMORY_UNIT_SIZE ${MEMORY_UNIT_SIZE})
math(EXPR MEMORY_UNIT_SIZE_BYTES "${MEMORY_UNIT_SIZE} / 8")
message(STATUS "Defined FPGA dual port unit size: ${MEMORY_UNIT_SIZE} bits (${MEMORY_UNIT_SIZE_BYTES} B)")

file(STRINGS ${ucp_filename} DEVICE_PART_FAMILY
    REGEX "DEVICE_PART_FAMILY=.*"
    )
string(REGEX REPLACE ".*DEVICE_PART_FAMILY=(.*)" "\\1" DEVICE_PART_FAMILY ${DEVICE_PART_FAMILY})

file(STRINGS ${ucp_filename} DEVICE_BOARD_NAME_CFG
    REGEX "BOARD_DEF=.*")
string(REGEX REPLACE ".*BOARD_DEF=(.*)" "\\1" DEVICE_BOARD_NAME_CFG ${DEVICE_BOARD_NAME_CFG})
message(STATUS "Board name: ${DEVICE_BOARD_NAME_CFG}")


#-------------------------------------------------------------------------------

add_custom_target(constraints_src
    SOURCES
        ${UCF_FILE_NAME_IN}
        ${ucp_filename}
    )

set(UCF_FILE_NAME           ${UCF_FILE_NAME}            PARENT_SCOPE)
set(USER_CODE_FLASH_OFFSET  ${USER_CODE_FLASH_OFFSET}   PARENT_SCOPE)
set(F_INPUTS_COUNT          ${F_INPUTS_COUNT}           PARENT_SCOPE)
set(GPIO_COUNT              ${GPIO_COUNT}               PARENT_SCOPE)
set(USE_PHISICAL_INPUTS     ${USE_PHISICAL_INPUTS}      PARENT_SCOPE)
set(MEMORY_UNIT_SIZE        ${MEMORY_UNIT_SIZE}         PARENT_SCOPE)
set(MEMORY_UNIT_SIZE_BYTES  ${MEMORY_UNIT_SIZE_BYTES}   PARENT_SCOPE)
set(DEVICE_PART_FAMILY      ${DEVICE_PART_FAMILY}       PARENT_SCOPE)

set(I2C_OVER_GPIO_SDA_PIN   ${I2C_OVER_GPIO_SDA_PIN}    PARENT_SCOPE)
set(I2C_OVER_GPIO_SCL_PIN   ${I2C_OVER_GPIO_SCL_PIN}    PARENT_SCOPE)
set(GPIO_PRESENT            ${GPIO_PRESENT}             PARENT_SCOPE)
set(DEVICE_BOARD_NAME_CFG   ${DEVICE_BOARD_NAME_CFG}    PARENT_SCOPE)
