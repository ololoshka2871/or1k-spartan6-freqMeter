#ifndef __IRQ_H__
#define __IRQ_H__

#include <stdint.h>

#include "mem_map.h"

#define ENTER_CRITICAL()        __or1k_disable_interrupts()
#define EXIT_CRITICAL()         __or1k_enable_interrupts()


typedef unsigned int* (*irq_handler)(unsigned int * registers); // global interrupt handler
typedef void (*isr_handler)(unsigned int * registers); // user interrupt

enum InterruptSources {
    IS_UART0 = IRQ_UART0,
    IS_TIMER_SYSTICK = IRQ_TIMER_SYSTICK,
    IS_TIMER_HIRES = IRQ_TIMER_HIRES,
    IS_BOOT_SPI = IRQ_BOOT_SPI,
    IS_GPIO = IRQ_GPIO,
    IS_MDIO = IRQ_MDIO,
    IS_I2C = IRQ_I2C,

    // ext
    IS_FREQMETERS = IRQ_FREQMETERS,
    IS_MINIMAC_TX = IRQ_MINIMAC_TX,
    IS_MINIMAC_RX = IRQ_MINIMAC_RX,

    IS_Count = IS_MINIMAC_RX + 1
};

//-----------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------
void irq_enable(int interrupt);
void irq_disable(int interrupt);
void irq_acknowledge(int interrupt);
int irq_check(int interrupt);

irq_handler install_irq_global_handler(irq_handler handler);
void interrupts_init(void);

void setInterruptPriority(enum InterruptSources src, uint8_t new_prio);
isr_handler set_irq_handler(enum InterruptSources src, isr_handler handler);

void __or1k_disable_interrupts(void);
void __or1k_enable_interrupts(void);

#endif
