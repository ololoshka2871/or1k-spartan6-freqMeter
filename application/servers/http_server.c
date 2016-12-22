/****************************************************************************
 *
 *   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
 *   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stddef.h>

#include "rodata.h"

#include "main.h"
#include "eth-http.h"

#include "ap-main.h"

#ifdef HTTP_AUTHORISE_REQUEST_FUNCTION
// autorisation
BYTE process_http_authorise_request (BYTE *requested_filename,
                                     BYTE *requested_filename_extension,
                                     BYTE tcp_socket_number) {
    return 1;
}
#endif

// templates values
BYTE *process_http_dynamic_data (BYTE *variable_name, BYTE tcp_socket_number) {
    return NULL;
}

// ???

void process_http_inputs (BYTE *input_name, BYTE *input_value,
                          BYTE *requested_filename,
                          BYTE *requested_filename_extension,
                          BYTE tcp_socket_number) {

}

#ifdef HTTP_ACCEPT_POST_REQUESTS
void process_http_multipart_form_header (const BYTE *input_name,
                                         BYTE *input_value,
                                         BYTE *requested_filename,
                                         BYTE *requested_filename_extension,
                                         BYTE tcp_socket_number) {

}

void process_http_multipart_form_last_section_done (void) {

}

void process_http_multipart_form_data(BYTE v) {

}
#endif

// FILES

BYTE process_http_find_file(BYTE* request_filename, BYTE* request_file_extension,
                            DWORD* file_size, DWORD* next_byte_address) {
    rodata_descriptor curent_file =
            rodata_find_file(request_filename, request_file_extension);
    if (curent_file == RODATA_INVALID_FILE_DESCRIPTOR) {
        return FALSE; // no file found
    }

    *file_size = rodata_filesize(curent_file);
    *next_byte_address = rodata_filedata_pointerAbsolute(curent_file);
    return TRUE;
}

BYTE process_http_file_next_byte(BYTE* pointer) {
    return rodata_readchar((uint32_t)pointer);
}

DWORD process_http_file_next_bytes(BYTE* buf, DWORD pointer, DWORD count) {
    return rodata_readarray_by_pointer(buf, (uint32_t)pointer, count);
}
