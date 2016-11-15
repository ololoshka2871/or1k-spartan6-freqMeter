#ifndef RODATA_H
#define RODATA_H

#include <stdint.h>

#define RODATA_INVALID_FILE_DESCRIPTOR          (~0)

typedef uint32_t rodata_descriptor;

rodata_descriptor rodata_find_file(char* name, char* ext);
uint32_t rodata_filesize(rodata_descriptor descriptor);
uint8_t rodata_readchar(rodata_descriptor descriptor, uint32_t offset);
uint32_t rodata_readarray(rodata_descriptor descriptor,
                          uint8_t* buf, uint32_t start, uint32_t size);

#endif // RODATA_H
