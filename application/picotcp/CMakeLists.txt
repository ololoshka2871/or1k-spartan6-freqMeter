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

set(picotcp_PARTCH          ${CMAKE_CURRENT_SOURCE_DIR}/0001-or1k-platform.patch)

find_package(Git REQUIRED)

ExternalProject_add(build_pico_tcp
    GIT_REPOSITORY ${picotcp_DISTRO}
    GIT_TAG master
    UPDATE_COMMAND
        rm -f include/arch/pico_or1k_gcc.h &&
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
            ARCH=or1k

    BUILD_IN_SOURCE 1
    INSTALL_COMMAND ""
    )

add_library(picotcp_core STATIC IMPORTED)
set_property(TARGET picotcp_core PROPERTY IMPORTED_LOCATION
        ${picotcp_BUILD_DIR}/lib/libpicotcp${CMAKE_STATIC_LIBRARY_SUFFIX})
add_dependencies(picotcp_core build_pico_tcp)

set(difenitions

    )

add_library(picotcp_port STATIC ${PICOTCP_PORT_SRC})

set_target_properties(picotcp_port PROPERTIES COMPILE_DEFINITIONS "${difenitions}")

set(PICOTCP_LIBNAME     ${CMAKE_CURRENT_BINARY_DIR}/libpicotcp${CMAKE_STATIC_LIBRARY_SUFFIX})

add_custom_command(
    OUTPUT ${PICOTCP_LIBNAME}
    COMMAND ar -rcT ${PICOTCP_LIBNAME}
        $<TARGET_FILE:picotcp_core> $<TARGET_FILE:picotcp_port>
    DEPENDS picotcp_core picotcp_port
    COMMENT "Combining libpicotcp"
    )

add_custom_target(picotcp_build DEPENDS libpicotcp${CMAKE_STATIC_LIBRARY_SUFFIX})

add_library(picotcp STATIC IMPORTED GLOBAL)
set_property(TARGET picotcp PROPERTY IMPORTED_LOCATION ${PICOTCP_LIBNAME})
add_dependencies(picotcp picotcp_build)

set(PICOTCP_INCLUDE_DIR      ${picotcp_BUILD_DIR}/include        PARENT_SCOPE)
