#ifndef __CRC32_H
#define __CRC32_H

#ifdef __cplusplus
extern "C" {
#endif

uint32_t crc32(const void* data, uint32_t length, uint32_t previousCrc32);

#ifdef __cplusplus
}
#endif

#endif
