#ifndef PROTOBUFPROTOCOL_H
#define PROTOBUFPROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

enum enProtobufResult {
    PB_OK = 0,
    PB_INPUT_MESSAGE_INCORRECT = 1,
};

enum enProtobufCMDFlags {
    PB_CMD_PONG = 0,
    PB_CMD_SETTINGS = 1 << 0,

};

typedef uint8_t (*protobuf_cb_input_data_reader)(uint8_t *buf, size_t count);
typedef void (*protobuf_cb_output_data_writer)(uint8_t *buf, size_t count);

enum enProtobufResult
protobuf_handle_request(protobuf_cb_input_data_reader reader,
                        enum enProtobufCMDFlags* pCmd_flags, uint32_t *pId);
void protobuf_format_error_message(protobuf_cb_output_data_writer writer);
void protobuf_format_answer(protobuf_cb_output_data_writer writer,
                            enum enProtobufCMDFlags cmd_flags, uint32_t id);

#endif // PROTOBUFPROTOCOL_H
