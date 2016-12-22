// //////////////////////////////////////////////////////////
// Crc32.cpp
// Copyright (c) 2011-2015 Stephan Brumme. All rights reserved.
// Slicing-by-16 contributed by Bulat Ziganshin
// see http://create.stephan-brumme.com/disclaimer.html
//

// g++ -o Crc32 Crc32.cpp -O3 -lrt -march=native -mtune=native

// if running on an embedded system, you might consider shrinking the
// big Crc32Lookup table:
// - crc32_bitwise doesn't need it at all
// - crc32_halfbyte has its own small lookup table
// - crc32_1byte    needs only Crc32Lookup[0]
// - crc32_4bytes   needs only Crc32Lookup[0..3]
// - crc32_8bytes   needs only Crc32Lookup[0..7]
// - crc32_4x8bytes needs only Crc32Lookup[0..7]
// - crc32_16bytes  needs all of Crc32Lookup

#include <stdint.h>

#include "mem_map.h"

/// compute CRC32 (half-byte algoritm)
static inline uint32_t _crc32(const void* data, uint32_t length)
{
  uint32_t crc = ~0; // same as 0xFFFFFFFF
  const uint8_t* current = (const uint8_t*) data;

  /// look-up table for half-byte, same as crc32Lookup[0][16*i]
  static const uint32_t Crc32Lookup16[16] =
  {
    0x00000000,0x1DB71064,0x3B6E20C8,0x26D930AC,0x76DC4190,0x6B6B51F4,0x4DB26158,0x5005713C,
    0xEDB88320,0xF00F9344,0xD6D6A3E8,0xCB61B38C,0x9B64C2B0,0x86D3D2D4,0xA00AE278,0xBDBDF21C
  };

  while (length-- != 0)
  {
    crc = Crc32Lookup16[(crc ^  *current      ) & 0x0F] ^ (crc >> 4);
    crc = Crc32Lookup16[(crc ^ (*current >> 4)) & 0x0F] ^ (crc >> 4);
    current++;
  }

  return ~crc; // same as crc ^ 0xFFFFFFFF
}

#ifdef CRC32_HW

#define CRC32_IO_REG                (*(REG32(HW_MAP_BASE)))
#define CRC32_RESET_REG             (*(REG32(HW_MAP_BASE + sizeof(uint32_t))))

/// Hardware CRC32 caclculator
uint32_t crc32(const void* data, uint32_t length) {
    CRC32_RESET_REG = 1; // reset

    const uint8_t* current = (const uint8_t*) data;

    while (length--) {
        CRC32_IO_REG = *current++;
    }

    return CRC32_IO_REG;
}
#else
uint32_t crc32(const void* data, uint32_t length) { return _crc32(data, length); }
#endif
