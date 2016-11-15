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

add_subdirectory(scripts)
add_subdirectory(siteroot)

set(site_blob_name      ${CMAKE_CURRENT_BINARY_DIR}/site.blob)

if (CMAKE_BUILD_TYPE EQUAL "Release")
    # TODO minimise code
    add_custom_command(
        OUTPUT ${site_blob_name}
        DEPENDS ${SITE_SRC} ${BLOB_GENERATOR_SCRIPT}
        COMMAND ${PYTHON_EXECUTABLE}
            ${BLOB_GENERATOR_SCRIPT}
            --outputblob ${site_blob_name}
            --path_max ${SITE_FILENAME_MAX}
            --ext_max ${SITE_FILEEXT_MAX}
            --ignore_regexp "CMakeLists"
            ${siteroot_DIR}
        )

    add_custom_target(site2blob DEPENDS ${site_blob_name})
else()
    add_custom_command(
        OUTPUT ${site_blob_name}
        DEPENDS ${SITE_SRC} ${BLOB_GENERATOR_SCRIPT}
        COMMAND ${PYTHON_EXECUTABLE}
            ${BLOB_GENERATOR_SCRIPT}
            --outputblob ${site_blob_name}
            --path_max ${SITE_FILENAME_MAX}
            --ext_max ${SITE_FILEEXT_MAX}
            --ignore_regexp "CMakeLists"
            ${siteroot_DIR}
        )

    add_custom_target(site2blob DEPENDS ${site_blob_name})
endif()

set(SITE_BLOB_NAME      ${site_blob_name}       PARENT_SCOPE)
