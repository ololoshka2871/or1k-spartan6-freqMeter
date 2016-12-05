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

#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "rtc.h"

#include "settings.h"

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

#include "protocol.pb.h"

#include "protobuf-protocol.h"


static bool RxCallback(pb_istream_t *stream, uint8_t *buf, size_t count) {
    protobuf_cb_input_data_reader reader = stream->state;

    if (!reader(buf, count)) {
        stream->bytes_left = 0; //eof
        return false;
    } else {
        return true;
    }
}


static bool TxCallback(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
    protobuf_cb_output_data_writer writer = stream->state;
    writer(buf, count);
    return true;
}


enum enProtobufResult
protobuf_handle_request(protobuf_cb_input_data_reader reader,
                        uint32_t *pId, struct sAnsverParameters* ansCookie) {
    ru_sktbelpa_r4_24_2_Request request;

    pb_istream_t input_stream = { RxCallback, reader, SIZE_MAX };
    if (!pb_decode(&input_stream, ru_sktbelpa_r4_24_2_Request_fields, &request)) {
        return PB_INPUT_MESSAGE_INCORRECT;
    }

    *pId = request.id;
    ansCookie->cmdFlags = PB_CMD_PONG;

    if (request.has_writeSettingsReq) {
        ansCookie->cmdFlags |= PB_CMD_SETTINGS;

        struct sSettings currentSettings;
        memcpy(&currentSettings, &settings, sizeof(struct sSettings));
        if (request.writeSettingsReq.has_setIPAddr)
            currentSettings.IP_addr.u32 = request.writeSettingsReq.setIPAddr;
        if (request.writeSettingsReq.has_setIPmask)
            currentSettings.IP_mask.u32 = request.writeSettingsReq.setIPmask;
        if (request.writeSettingsReq.has_setIPDefaultGateway)
            currentSettings.IP_gateway.u32 = request.writeSettingsReq.setIPDefaultGateway;
        if (request.writeSettingsReq.has_setMAC_Addr)
            memcpy(currentSettings.MAC_ADDR,
                   ((uint8_t*)&request.writeSettingsReq.setMAC_Addr) +
                        (sizeof(uint64_t) - MAC_ADDRESS_SIZE), MAC_ADDRESS_SIZE);
        if (request.writeSettingsReq.has_setUseDHCP)
            currentSettings.DHCP = request.writeSettingsReq.setUseDHCP;

        if ((ansCookie->settingResult =
                Settings_validate(&currentSettings, Settings_read)) == SV_ERR_OK) {
            // ok
            memcpy(&settings, &currentSettings, sizeof(struct sSettings));
            Settings_write(&settings);
            // to apply network settings reboot needed
        }
    }

    return PB_OK;
}


static void fill_generic_fields(ru_sktbelpa_r4_24_2_Response *responce) {
    clock_gettime(0, &responce->timestamp);
    responce->version = ru_sktbelpa_r4_24_2_INFO_PROTOCOL_VERSION;
}


static void sendResponce(protobuf_cb_output_data_writer writer,
                         ru_sktbelpa_r4_24_2_Response *responce) {
    pb_ostream_t output_stream = { TxCallback, writer, SIZE_MAX, 0 };
    assert(pb_encode(&output_stream, ru_sktbelpa_r4_24_2_Response_fields, responce));
}


void protobuf_format_error_message(protobuf_cb_output_data_writer writer) {
    ru_sktbelpa_r4_24_2_Response errresponce;

    fill_generic_fields(&errresponce);
    errresponce.id = ~0;
    errresponce.Global_status = ru_sktbelpa_r4_24_2_STATUS_PROTOCOL_ERROR;
    errresponce.has_settings = false;

    sendResponce(writer, &errresponce);
}


void protobuf_format_answer(protobuf_cb_output_data_writer writer, uint32_t id,
                            struct sAnsverParameters* args) {
    ru_sktbelpa_r4_24_2_Response responce;

    fill_generic_fields(&responce);
    responce.id = id;
    responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_OK;

    if ((responce.has_settings = (args->cmdFlags & PB_CMD_SETTINGS))) {
        if ((responce.settings.status = args->settingResult) !=
                ru_sktbelpa_r4_24_2_SettingsResponse_ErrorDescription_OK) {
            responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_ERRORS_IN_SUBCOMMANDS;
        }
        responce.settings.IPAddr = settings.IP_addr.u32;
        responce.settings.IPmask = settings.IP_mask.u32;
        responce.settings.IPDefaultGateway = settings.IP_gateway.u32;
        memset(&responce.settings.MAC_Addr, 0, sizeof(uint64_t) - MAC_ADDRESS_SIZE);
        memcpy(((uint8_t*)&responce.settings.MAC_Addr) +
                    (sizeof(uint64_t) - MAC_ADDRESS_SIZE),
                        settings.MAC_ADDR, MAC_ADDRESS_SIZE);
        responce.settings.UseDHCP = settings.DHCP;
    }

    sendResponce(writer, &responce);
}
