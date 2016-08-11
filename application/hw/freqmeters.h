#ifndef FREQMETERS_H
#define FREQMETERS_H

#include <stdint.h>

#include "mem_map.h"

#ifndef FREQMETERS_COUNT
#define FREQMETERS_COUNT    24
#warning "Macro FREQMETERS_COUNT not defined! Assuming 24"
#endif

#define FREQMETERS_START_SELECTOR       (1 << 5)
#define FREQMETERS_MEMORY_SELECTOR      (1 << 6)

// 0x11000060
#define FM_START_VALS_BASE      (FREQMETERS_BASE | FREQMETERS_MEMORY_SELECTOR | FREQMETERS_START_SELECTOR)
// 0x11000040
#define FM_STOP_VALS_BASE       (FREQMETERS_BASE | FREQMETERS_MEMORY_SELECTOR)
// 0x11000020
#define FM_RELOADINGS_BASE      (FREQMETERS_BASE | FREQMETERS_START_SELECTOR)
// 0x11000000
#define FM_IE                   (*(REG32(FREQMETERS_BASE + 0)))
// 0x11000001
#define FM_IF                   (*(REG32(FREQMETERS_BASE + 1)))

#define FM_START_VAL_CH(chanel) (*(REG32(FM_START_VALS_BASE + chanel * sizeof(uint32_t))))
#define FM_STOP_VAL_CH(chanel)  (*(REG32(FM_STOP_VALS_BASE + chanel * sizeof(uint32_t))))
#define FM_RELOAD_CH(chanel, v) (*(REG32(FM_RELOADINGS_BASE + chanel * sizeof(uint32_t))) = v)

struct freqmeter_chanel {
    uint32_t newReload_vals[3]; // 0 - new, 1 - in_work, 2 - ready
    uint32_t res_start_v;
    uint32_t res_stop_v;

    unsigned enabled:1;
};

void fm_init();

void update_chanel(uint8_t chanel);

#endif // FREQMETERS_H
