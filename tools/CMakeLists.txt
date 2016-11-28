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

#include(ExternalProject)
#include(ProcessorCount)

#ProcessorCount(make_jobs)

#set(srecord_DIR	    ${CMAKE_CURRENT_BINARY_DIR}/srecord)

#ExternalProject_Add(srecord
#    DOWNLOAD_DIR    ${CMAKE_CURRENT_BINARY_DIR}
#    URL		    http://downloads.sourceforge.net/project/srecord/srecord/1.64/srecord-1.64.tar.gz
#    URL_MD5	    4de4a7497472d7972645c2af91313769
#    BUILD_IN_SOURCE 1
#    CONFIGURE_COMMAND ./configure --prefix=${srecord_DIR}
#    BUILD_COMMAND   ${CMAKE_MAKE_PROGRAM} -j${make_jobs}
#    INSTALL_DIR	    ${srecord_DIR}
#    )

#set(utils	cat cmp info)
#foreach(u ${utils})
#    find_file(
#	srec_${u}
#	${u}
#	PATHS
#	    ${srecord_DIR}/bin
#	)
#endforeach(u)

add_custom_target(tools.editor
    SOURCES
        filter_debug_io_log.py
        mkgdb_server.sh
        merge_binary.py
        calc_best_signal_test_devider.py
        decodeDoubleDump.py
        decodeChanels.py
        test_data_querryer.py
        gen-callgraph.py
        ws_tester.html
    )

set(tools_DIR       ${CMAKE_CURRENT_SOURCE_DIR}         PARENT_SCOPE)
