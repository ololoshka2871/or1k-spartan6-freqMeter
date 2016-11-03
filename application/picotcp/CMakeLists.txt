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

include(ExternalProject)

set(PICOTCP_PORT_SRC
    eth_driver.c
    )

set(picotcp_DISTRO          ${CMAKE_CURRENT_SOURCE_DIR}/dist_picotcp)
set(picotcp_BUILD_DIR       ${CMAKE_CURRENT_BINARY_DIR})

set(picotcp_PARTCH          ${CMAKE_CURRENT_SOURCE_DIR}/0001-or1k-system-definitions.patch)

find_package(Git REQUIRED)

ExternalProject_add(build_pico_tcp
    GIT_REPOSITORY ${picotcp_DISTRO}
    GIT_TAG master
    UPDATE_COMMAND
        ${GIT_EXECUTABLE} reset --hard HEAD &&
        ${GIT_EXECUTABLE} apply --ignore-whitespace ${picotcp_PARTCH}

    CONFIGURE_COMMAND ""
    BUILD_COMMAND
        ${CMAKE_MAKE_PROGRAM}
            CROSS_COMPILE=${TOOLCHAIN_PREFIX}
            PREFIX=${picotcp_BUILD_DIR}
            ENDIAN=big
            PLATFORM_CFLAGS=${CMAKE_C_FLAGS_COMMON}
            ADDRESS_SANITIZER=0
            ARCH=OR1K

    BUILD_IN_SOURCE 0
    )

set(difenitions

    )

add_library(picotcp STATIC ${PICOTCP_PORT_SRC})

set_target_properties(picotcp PROPERTIES COMPILE_DEFINITIONS "${difenitions}")

#set(LIBHW_INCLUDE_DIR           ${CMAKE_CURRENT_SOURCE_DIR}      PARENT_SCOPE)
