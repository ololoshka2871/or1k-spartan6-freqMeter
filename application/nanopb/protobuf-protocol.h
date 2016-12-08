#ifndef PROTOBUFPROTOCOL_H
#define PROTOBUFPROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "settings.h"

enum enProtobufResult {
    PB_OK = 0,
    PB_INPUT_MESSAGE_INCORRECT = 1,
};

enum enProtobufCMDFlags {
    PB_CMD_PONG = 0,
    PB_CMD_SETTINGS = 1 << 0,
    PB_CMD_SETCLOCK = 1 << 1,
};

struct sAnsverParameters {
    enum enProtobufCMDFlags cmdFlags;

    uint32_t settingResult; // result of execute settings set
    uint32_t settimeResult; // result of execute set clock
};

typedef uint8_t (*protobuf_cb_input_data_reader)(uint8_t *buf, size_t count);
typedef void (*protobuf_cb_output_data_writer)(uint8_t *buf, size_t count);

enum enProtobufResult
protobuf_handle_request(protobuf_cb_input_data_reader reader, uint32_t *pId,
                        struct sAnsverParameters* ansCookie);
void protobuf_format_error_message(protobuf_cb_output_data_writer writer);
void protobuf_format_answer(protobuf_cb_output_data_writer writer, uint32_t id,
                            struct sAnsverParameters* args);

#endif // PROTOBUFPROTOCOL_H
