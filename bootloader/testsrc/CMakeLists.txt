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

file(GLOB_RECURSE
    TEST_SRC
    FOLLOW_SYMLINKS
    "*.c"
    )

add_definitions(
    -DBOOTLOADER
    -DSIM
    -DF_REF=${REFERENCE_CLOCK_HZ}
    -DFREQMETERS_COUNT=24
    -DF_CPU=${CPU_CLOCK_HZ}
    -DBB_I2C_BAUD=${BAUD_I2C}
    -DBB_I2C_SDA_PIN=6
    -DBB_I2C_SCL_PIN=7
    -DSYSTEM_FREQ_TYPE=${SYSTEM_FREQ_TYPE}
    )

if(PERIPHERIAL_ENABLE_I2C)
    add_definitions(-DENABLE_I2C=1)
else()
    add_definitions(-DENABLE_I2C=0)
endif()

if(PERIPHERIAL_ENABLE_GPIO)
    add_definitions(-DENABLE_GPIO=1)
else()
    add_definitions(-DENABLE_GPIO=0)
endif()

if (PERIPHERIAL_ENABLE_I2C)
     add_definitions(-DIICMB_I2C)
endif()

if (PERIPHERIAL_ENABLE_CRC32)
    add_definitions(-DCRC32_HW)
endif()

if (PERIPHERIAL_ENABLE_HW_MUL)
    add_definitions(-DHW_MUL_MODULE)
endif()

if (SIM_TEST_FREQMETER)
    add_definitions(-DSIM_TEST_FREQMETER)
endif()

if (SIM_TEST_MULTIPLICATION)
    add_definitions(-DSIM_TEST_MULTIPLICATION)
endif()

if (SIM_TEST_MINIMAC_SLOT_LOGICK)
    add_definitions(-DSIM_TEST_MINIMAC_SLOT_LOGICK)
endif()

if (SIM_TEST_MINIMAC)
    add_definitions(-DSIM_TEST_MINIMAC)
endif()

if (SIM_TEST_MDIO)
    add_definitions(-DSIM_TEST_MDIO)
endif()

if (SIM_TEST_I2C)
    add_definitions(-DSIM_TEST_I2C)
endif()

if (SIM_TEST_GPIO)
    add_definitions(-DSIM_TEST_GPIO)
endif()

if (SIM_TEST_CRC32)
    add_definitions(-DSIM_TEST_CRC32)
endif()

include_directories(. ${GDB_STUB_DIR})
add_library(testsrc STATIC ${TEST_SRC})
