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
#include <syscall.h>

#include "main.h"
#include "eth-main.h"
#include "eth-nic.h"
#include "eth-udp.h"

#include "rtc.h"

#include "settings.h"

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

#include "protocol.pb.h"

#include "protobuf-protocol.h"


static bool RxCallback(pb_istream_t *stream, uint8_t *buf, size_t count) {
    (void)stream;
    return nic_read_array(buf, count);
}


static bool TxCallback(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
    uint8_t* pwr = stream->state;
    if (count == 1)
        *pwr = *buf;
    else
        memcpy(pwr, buf, count);
    stream->state += count;
    return true;
}


enum enProtobufResult
protobuf_handle_request(uint16_t rx_data_bytes_remaining,
                        struct sAnsverParameters* ansCookie) {
    ru_sktbelpa_r4_24_2_Request request;

    pb_istream_t input_stream = { RxCallback, NULL, rx_data_bytes_remaining };
    if (!pb_decode(&input_stream, ru_sktbelpa_r4_24_2_Request_fields, &request)) {
        return PB_INPUT_MESSAGE_INCORRECT;
    }

    if ((request.deviceID != ru_sktbelpa_r4_24_2_INFO_R4_24_2_ID) &&
            (request.deviceID != ru_sktbelpa_r4_24_2_INFO_ID_DISCOVER))
        return PB_SKIP;

    ansCookie->id = request.id;

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

        if (request.writeSettingsReq.has_setReferenceFrequency)
            currentSettings.ReferenceFrequency = request.writeSettingsReq.setReferenceFrequency;

        settings_update_crc32(&currentSettings);
        if ((ansCookie->settingResult =
                Settings_validate(&currentSettings, Settings_read)) == SV_ERR_OK) {
            memcpy(&settings, &currentSettings, sizeof(struct sSettings));
            Settings_write(&settings);
            // to apply network settings reboot needed
        }
    }

    if (request.has_setClock) {
        ansCookie->cmdFlags |= PB_CMD_SETCLOCK;
        ansCookie->settimeResult = clock_settime(0, (struct timespec*)&request.setClock);
    }

    if (request.has_setMeasureTimeRequest) {
        ansCookie->cmdFlags |= PB_CMD_SETMEASURE_TIME;
        ru_sktbelpa_r4_24_2_SetMeasureTime_message* chanelSetMeasureTime =
                request.setMeasureTimeRequest.chanelSetMeasureTime;
        struct sSetMeasureTimeresult* setMeasureTimeResults = ansCookie->setMeasureTimeResults;
        for(uint32_t i = 0; i < FREQMETERS_COUNT; ++i) {
            if (i < request.setMeasureTimeRequest.chanelSetMeasureTime_count) {
                ru_sktbelpa_r4_24_2_SetMeasureTime_message * current = &chanelSetMeasureTime[i];
                struct sSetMeasureTimeresult* current_res = &setMeasureTimeResults[i];
                uint32_t ch = current->chanelNumber;
                current_res->chanel = ch;
                if (ch >= FREQMETERS_COUNT) {
                    current_res->chanel = 0;
                    current_res->result = ERR_MT_INVALID_CHANEL;
                } else {
                    current_res->result = ERR_MT_OK;
                    if (current->has_measureTime_ms) {
                        current_res->result = fm_setMeasureTime(ch, current->measureTime_ms);
                    }
                    if (current->has_chanelEnabled) {
                        fm_enableChanel(ch, current->chanelEnabled);
                    }
                }
            } else {
                setMeasureTimeResults[i].chanel = -1;
            }
        }
    }

    if (request.has_getMeasureResultsReq) {
        ansCookie->cmdFlags |= PB_CMD_GETRESULTS;
        struct sMeasureResultCmdOpts* result_send = &ansCookie->result_send;

        result_send->chanels_result_send_mask = 0;
        result_send->Request_valid = true;

        const pb_size_t chanels_count = request.getMeasureResultsReq.chanels_count;
        for (uint i = 0; i < chanels_count; ++i) {
            ru_sktbelpa_r4_24_2_GetMeasureResult_message * current_chanel =
                    &request.getMeasureResultsReq.chanels[i];
            uint32_t ch = current_chanel->chanelNumber;
            if (ch >= FREQMETERS_COUNT) {
                result_send->Request_valid = false;
                continue;
            }
            uint32_t chanel_bit = 1 << ch;
            result_send->chanels_result_send_mask |= chanel_bit;
        }
    }

    // reboot
    if (request.has_rebootRequest) {
        if (request.rebootRequest.resetDefaults) {
            settings_defaults(&settings); // reset settings to default
            Settings_write(&settings);
        }
        reboot();
    }

    return PB_OK;
}


static void fill_generic_fields(ru_sktbelpa_r4_24_2_Response *responce) {
    memset(responce, 0, sizeof(ru_sktbelpa_r4_24_2_Response));
    clock_gettime(0, (struct timespec*)&responce->timestamp);
    responce->deviceID = ru_sktbelpa_r4_24_2_INFO_R4_24_2_ID;
    responce->protocolVersion = ru_sktbelpa_r4_24_2_INFO_PROTOCOL_VERSION;
}


static void sendResponce(ru_sktbelpa_r4_24_2_Response *responce) {
    uint8_t* wrpos = nic_get_wrpointer();
    pb_ostream_t output_stream = { TxCallback, wrpos, SIZE_MAX, 0 };
    assert(pb_encode(&output_stream, ru_sktbelpa_r4_24_2_Response_fields, responce));
    udp_writen_directly(wrpos, output_stream.bytes_written);
}


void protobuf_format_error_message() {
    ru_sktbelpa_r4_24_2_Response errresponce;

    fill_generic_fields(&errresponce);
    errresponce.id = ~0;
    errresponce.Global_status = ru_sktbelpa_r4_24_2_STATUS_PROTOCOL_ERROR;

    sendResponce(&errresponce);
}


void protobuf_format_answer(struct sAnsverParameters* args) {
    ru_sktbelpa_r4_24_2_Response responce;

    fill_generic_fields(&responce);
    responce.id = args->id;
    responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_OK;

    if ((responce.has_settings = (args->cmdFlags & PB_CMD_SETTINGS))) {
        ru_sktbelpa_r4_24_2_SettingsResponse * psettings = &responce.settings;

        if ((psettings->status = args->settingResult) !=
                ru_sktbelpa_r4_24_2_SettingsResponse_ErrorDescription_OK) {
            responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_ERRORS_IN_SUBCOMMANDS;
        }
        psettings->IPAddr = settings.IP_addr.u32;
        psettings->IPmask = settings.IP_mask.u32;
        psettings->IPDefaultGateway = settings.IP_gateway.u32;

        memset(&psettings->MAC_Addr, 0, sizeof(uint64_t) - MAC_ADDRESS_SIZE);
        memcpy(((uint8_t*)&psettings->MAC_Addr) + (sizeof(uint64_t) - MAC_ADDRESS_SIZE),
               settings.MAC_ADDR, MAC_ADDRESS_SIZE);

        psettings->UseDHCP = settings.DHCP;

        psettings->ReferenceFrequency = settings.ReferenceFrequency;
    }

    if ((args->cmdFlags & PB_CMD_SETCLOCK) && (args->settimeResult)) {
        responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_ERRORS_IN_SUBCOMMANDS;
    }

    if ((responce.has_getMeasureTimeResponce = (args->cmdFlags & PB_CMD_SETMEASURE_TIME))) {
        ru_sktbelpa_r4_24_2_GetMeasureTimeResponce *result = &responce.getMeasureTimeResponce;

        result->chanelgetMeasureTime_count = 0;
        for(uint32_t i = 0; i < FREQMETERS_COUNT; ++i) {
            struct sSetMeasureTimeresult* chanel_req_info = &args->setMeasureTimeResults[i];
            ru_sktbelpa_r4_24_2_GetMeasureTime_message *chanelgetMeasureTime_item = &result->chanelgetMeasureTime[i];

            if (chanel_req_info->chanel >= 0) {
                chanelgetMeasureTime_item->chanelNumber = chanel_req_info->chanel;
                ++result->chanelgetMeasureTime_count;
                chanel_req_info->result |=
                        fm_getMeasureTime_ms(chanel_req_info->chanel, &chanelgetMeasureTime_item->measureTime_ms);

                if ((enum enSetMeasureTimeError)(chanelgetMeasureTime_item->status = chanel_req_info->result)
                        != ERR_MT_OK) {
                    responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_ERRORS_IN_SUBCOMMANDS;
                    chanelgetMeasureTime_item->chanelEnabled = false;
                } else {
                    chanelgetMeasureTime_item->chanelEnabled = fm_isChanelEnabled(chanel_req_info->chanel);
                }
            }
        }
    }

    if ((responce.has_getMeasureResultsResponce = (args->cmdFlags & PB_CMD_GETRESULTS))) {
        const uint32_t chanels_result_send_mask = args->result_send.chanels_result_send_mask;
        ru_sktbelpa_r4_24_2_GetMeasureResultsResponce* getMeasureResultsResponce = &responce.getMeasureResultsResponce;
        ru_sktbelpa_r4_24_2_MeasureResult* results = getMeasureResultsResponce->results;
        pb_size_t* results_count = &getMeasureResultsResponce->results_count;

        *results_count = 0;
        for (uint chanel = 0; chanel < FREQMETERS_COUNT; ++chanel) {
            if (chanels_result_send_mask & (1 << chanel)) {
                ru_sktbelpa_r4_24_2_MeasureResult *res_item = &results[*results_count];
                struct freqmeter_chanel chanel_data;

                fm_getCopyOffreqmeterState(chanel, &chanel_data);
                res_item->chanelNumber = chanel;
                memcpy(&res_item->timestamp, &chanel_data.timestamp, sizeof(struct timespec));
                clock_purify_time((struct timespec*)&res_item->timestamp);
                res_item->chanelEnabled = chanel_data.enabled;
                res_item->chanelInputSignalPresent = chanel_data.signal_present;
                res_item->Frequency = chanel_data.F;
                ++(*results_count);
            }
        }

        if ((getMeasureResultsResponce->status = !args->result_send.Request_valid))
            responce.Global_status = ru_sktbelpa_r4_24_2_STATUS_ERRORS_IN_SUBCOMMANDS;
    }

    sendResponce(&responce);
}
