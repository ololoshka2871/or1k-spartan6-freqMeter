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

set(SETTINGS_SRC                    settings.c)
set(ETHERNET_CONFIG_INCLUDE_FILE    ${CMAKE_CURRENT_BINARY_DIR}/ETH_config.h)

if(ETHERNET_MAC_ADDRESS_FORCE)
    set(_gen_MAC        ${ETHERNET_MAC_ADDRESS_FORCED})
else(ETHERNET_MAC_ADDRESS_FORCE)
    set(_gen_MAC        ${ETHERNET_MAC_ADDRESS_MSB})
endif(ETHERNET_MAC_ADDRESS_FORCE)

set(_genMAC_ARGUMENTS
    ${_gen_MAC}
    ${ETHERNET_STATIC_IP_ADDR}
    ${ETHERNET_STATIC_IP_NETMASK}
    ${ETHERNET_STATIC_IP_GATEWAY}
    )

SET(LIBHW_SOURCES
    cache.c
    irq.c
    serial0.c
    timer.c
    GPIO.c
    syscall.c
    seg7_display.c
    freqmeters.c
    minimac2.c
    mdio.c
    prog_timer.c
    rodata.c
    iicmb.c
    bb_i2c.c
    i2c.c
    ds1338z.c
    rtc.c
    ${SETTINGS_SRC}
    crc32.c
    )

add_custom_command(
    OUTPUT ${ETHERNET_CONFIG_INCLUDE_FILE}
    COMMAND
        ${GEN_ETH_CONFIG} ${_genMAC_ARGUMENTS} > ${ETHERNET_CONFIG_INCLUDE_FILE}
    DEPENDS ${MACADDR_GENERATOR}
    COMMENT
        "Generating random MAC address"
    )
add_custom_target(gen_mac DEPENDS ${ETHERNET_CONFIG_INCLUDE_FILE})

add_custom_target(regen_mac
    COMMAND
        rm ${ETHERNET_CONFIG_INCLUDE_FILE} && touch ${SETTINGS_SRC}
    COMMENT
        "Force regenerate MAC address on next build"
    )

if(PERIPHERIAL_ENABLE_HW_MUL)
    list(APPEND LIBHW_SOURCES __mulsi3.c)
endif()

include_directories(. ${CMAKE_CURRENT_BINARY_DIR} ${LIBBASE_INCLUDE_DIR})

execute_process(COMMAND date +%s
                OUTPUT_VARIABLE b_timestamp
                OUTPUT_STRIP_TRAILING_WHITESPACE
                )
message(STATUS "Build timestamp: ${b_timestamp}")

set(difenitions
    MAC_TX_MEM_BASE=${MAC_TX_MEM_BASE}
    MAC_RX_MEM_BASE=${MAC_RX_MEM_BASE}
    MAC_CTL_BASE=${MAC_CTL_BASE}
    MTU=${MTU}
    MYMINMAC_RX_SLOTS=${MYMINMAC_RX_SLOTS}

    FREQMETER_MEASURE_TIME_MAX=${FREQMETER_MEASURE_TIME_MAX}
    FREQMETER_MEASURE_TIME_MIN=${SYSTEM_MEASURE_TIME_MIN}
    FREQMETER_MASTER_CLOCK_RATIO=${CLOCK_FREF_PLL_MULTIPLYER}/${CLOCK_FREF_CLOCK_DEVIDER}
    SYSTEM_MEASURE_TIME_DEFAULT=${SYSTEM_MEASURE_TIME_DEFAULT}

    RODATA_BLOB_START=${SITE_BLOB_START}
    RODATA_FILENAME_LEN=${SITE_FILENAME_MAX}
    RODATA_FILEEXT_LEN=${SITE_FILEEXT_MAX}
    RODATA_CACHE_SIZE=${SITE_STATIC_CACHE}

    BB_I2C_PORT=GPIO_PORTA
    BB_I2C_SDA_PIN=${I2C_OVER_GPIO_SDA_PIN}
    BB_I2C_SCL_PIN=${I2C_OVER_GPIO_SCL_PIN}

    GPIO_ENABLED=${GPIO_PRESENT}

    DEVICE_REF_CLOCK_HZ=${DEVICE_REF_CLOCK_HZ}
    FREF_PLL_MULTIPLYER=${FREF_PLL_MULTIPLYER}
    FREF_CLOCK_DEVIDER=${FREF_CLOCK_DEVIDER}
    BULD_TIMESTAMP=${b_timestamp}
    )

if(MASTER_HYBRID_COUNTER)
    list(APPEND difenitions MASTER_HYBRID_COUNTER)
endif()

if(PERIPHERIAL_ENABLE_I2C)
    list(APPEND difenitions ENABLE_I2C=1)
else()
    list(APPEND difenitions ENABLE_I2C=0)
endif()

if(PERIPHERIAL_ENABLE_GPIO)
    list(APPEND difenitions ENABLE_GPIO=1)
else()
    list(APPEND difenitions ENABLE_GPIO=0)
endif()

if(ETHERNET_USE_DHCP)
    list(APPEND difenitions DHCP_ON_STARTUP)
endif()

if (ETHERNET_MAC_ADDRESS_FORCE)
    list(APPEND difenitions MAC_ADDR_FORCE)
endif()

if (PERIPHERIAL_ENABLE_CRC32)
    list(APPEND difenitions CRC32_HW)
endif()

add_library(hw STATIC ${LIBHW_SOURCES})

add_dependencies(hw gen_mac)

set_target_properties(hw PROPERTIES COMPILE_DEFINITIONS "${difenitions}")

set(LIBHW_INCLUDE_DIR           ${CMAKE_CURRENT_SOURCE_DIR}      PARENT_SCOPE)
