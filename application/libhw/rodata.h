#ifndef RODATA_H
#define RODATA_H

#include <stdint.h>

#define RODATA_INVALID_FILE_DESCRIPTOR          NULL

struct rodata_file;

typedef struct rodata_file* rodata_descriptor;

rodata_descriptor rodata_find_file(char* name, char* ext);
uint32_t rodata_filesize(rodata_descriptor descriptor);
uint32_t rodata_filedata_pointerAbsolute(rodata_descriptor descriptor);
uint8_t rodata_readchar(uint32_t offset);
uint32_t rodata_readarray(rodata_descriptor descriptor,
                          uint8_t* buf, uint32_t start, uint32_t size);
uint32_t rodata_readarray_by_pointer(uint8_t *buf, uint32_t start, uint32_t size);

#endif // RODATA_H
