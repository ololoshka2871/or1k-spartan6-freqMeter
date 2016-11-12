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

set(MK_APP_LD_SCRIPT   "${CMAKE_CURRENT_SOURCE_DIR}/mkldscript.sh")
set(SUMMARY_GENERATOR  "${CMAKE_CURRENT_SOURCE_DIR}/mksummary.sh")
set(GEN_ETH_CONFIG     "${CMAKE_CURRENT_SOURCE_DIR}/gen_eth_config.sh")

add_custom_target(app_scripts.editor
    SOURCES
        ${MK_APP_LD_SCRIPT}
        ${SUMMARY_GENERATOR}
        ${GEN_ETH_CONFIG}
        elf_get_section_info.sh
    )
    
set(MK_APP_LD_SCRIPT    ${MK_APP_LD_SCRIPT}             PARENT_SCOPE)
set(SUMMARY_GENERATOR   ${SUMMARY_GENERATOR}            PARENT_SCOPE)
set(GEN_ETH_CONFIG      ${GEN_ETH_CONFIG}               PARENT_SCOPE)

