#ifndef PROTOBUFPROTOCOL_H
#define PROTOBUFPROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

enum enProtobufResult {
    PB_OK = 0,
    PB_INPUT_MESSAGE_INCORRECT = 1,
};

typedef uint8_t (*protobuf_cb_input_data_reader)(uint8_t *buf, size_t count);

enum enProtobufResult
protobuf_handle_request(protobuf_cb_input_data_reader reader);

#endif // PROTOBUFPROTOCOL_H
