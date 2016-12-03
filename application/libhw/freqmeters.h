#ifndef FREQMETERS_H
#define FREQMETERS_H

#include <stdint.h>
#include <stdbool.h>
#ifndef SIM
#include "rtc.h"
#endif
#include "mem_map.h"

#ifndef FREQMETERS_BASE
#warning "FREQMETERS_BASE undefined"
#define FREQMETERS_BASE         (FIO_BASE + 0x00000000)
#endif

#ifndef FREQMETERS_COUNT
#define FREQMETERS_COUNT    24
#warning "Macro FREQMETERS_COUNT not defined! Assuming 24"
#endif

#ifndef FREQMETERS_MASTER_COUNT_LEN
#define FREQMETERS_MASTER_COUNT_LEN    30
#warning "Macro FREQMETERS_MASTER_COUNT_LEN not defined! Assuming 30"
#endif

#ifndef F_REF
#error "Macro F_REF not defined!"
#endif

#define ALIGNMENT_SHIFT(x)              (x + 2)

#define FREQMETERS_START_SELECTOR       (1 << ALIGNMENT_SHIFT(5))

#define FM_START_VALS_BASE      (FREQMETERS_BASE + 0x100)
// 0x11000100
#define FM_STOP_VALS_BASE       (FREQMETERS_BASE + 0x200)
// 0x11000080
#define FM_RELOADINGS_BASE      (FREQMETERS_BASE | FREQMETERS_START_SELECTOR)
// 0x11000000
#define FM_IE                   (*(REG32(FREQMETERS_BASE + 0)))
// 0x11000004
#define FM_IF                   (*(REG32(FREQMETERS_BASE + sizeof(uint32_t))))
// 0x11000008
#define FM_SP                   (*(REG32(FREQMETERS_BASE + (2 * sizeof(uint32_t)))))


#define FM_START_VAL_CH(chanel) (*(REG32(FM_START_VALS_BASE + (chanel) * sizeof(uint32_t))))
#define FM_STOP_VAL_CH(chanel)  (*(REG32(FM_STOP_VALS_BASE + (chanel) * sizeof(uint32_t))))
#define FM_RELOAD_CH(chanel, v) (*(REG32(FM_RELOADINGS_BASE + (chanel) * sizeof(uint32_t))) = (v))

struct freqmeter_chanel {
    struct {
        uint32_t newReload_val;
        uint32_t inWorkReload_val;
        uint32_t readyReload_val;
    } reloadVals;
    uint32_t res_start_v;
    uint32_t res_stop_v;
    uint32_t irq_count;
#ifndef SIM
    struct timespec timestamp;
#endif
    unsigned enabled:1;
    unsigned signal_present:1;
} __attribute__((packed));

void fm_init();

void fm_updateChanel(uint8_t chanel);
void fm_enableChanel(uint8_t chanel, bool enable);
void fm_setChanelReloadValue(uint8_t chanel, uint32_t reload_value,
                             bool force_restart);
uint32_t fm_getActualMeasureTime(uint8_t chanel);
uint32_t fm_getActualReloadValue(uint8_t chanel);
uint32_t fm_getMeasureTimestamp(uint8_t chanel);
uint32_t fm_getMeasureStart_pos(uint8_t chanel);
uint32_t fm_getIRQCount(uint8_t chanel);
bool     fm_checkAlive(uint8_t chanel);

#endif // FREQMETERS_H
