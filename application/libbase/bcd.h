#ifndef BCD_H
#define BCD_H

#include <stdint.h>

// transform decimal value to bcd
uint8_t u8_dec2bcd(uint8_t val);

//transform bcd value to deciaml
uint8_t u8_bcd2dec(uint8_t val);

#endif // BCD_H
