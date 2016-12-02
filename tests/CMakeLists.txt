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

set(PB_TESTER           ${CMAKE_CURRENT_SOURCE_DIR}/pb_tester.py)

add_custom_target(tests.editor
    SOURCES
        decodeDoubleDump.py
        decodeChanels.py
        test_data_querryer.py
        ws_tester.html
        ${PB_TESTER}
    )

find_program(PROTOC_EXECUTABLE protoc)

string(REGEX REPLACE "(.*)\\.proto" "\\1_pb2.py" protocol_python_module ${PROTOBUF_PROTO_FILE})
get_filename_component(protocol_proto_dir ${protocol_python_module} DIRECTORY)

if (PROTOC_EXECUTABLE)
    if (PYTHON_VERSION_MAJOR STRGREATER "2")
        message(STATUS "Forsing python2 for nanopb")
        find_program(PYTHON_2   python2)
        if (NOT PYTHON_2)
            message(STATUS "python2 not found, U can try use python3, but it was not works for me")
        else()
            set(PYTHON_EXECUTABLE ${PYTHON_2})
        endif()
    endif()

    add_custom_command(
        OUTPUT ${protocol_python_module}
        COMMAND ${PROTOC_EXECUTABLE}
            -I${protocol_proto_dir}
            --python_out=${CMAKE_CURRENT_BINARY_DIR}
            ${PROTOBUF_PROTO_FILE}
        DEPENDS ${PROTOBUF_PROTO_FILE}
        )

    add_custom_target(pb_tesetr.run
        COMMAND
            PYTHONPATH=. ${PYTHON_EXECUTABLE} ${PB_TESTER}
        DEPENDS
            ${PB_TESTER} ${protocol_python_module}
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
        )
else()
    message(STATUS "protoc PROTOBUF compiler not found, protobuf test will not work")
endif()

set(tests_DIR       ${CMAKE_CURRENT_SOURCE_DIR}         PARENT_SCOPE)
