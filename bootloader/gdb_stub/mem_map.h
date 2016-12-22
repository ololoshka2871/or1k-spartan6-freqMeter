#ifndef __MEM_MAP_H__
#define __MEM_MAP_H__

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define IO_BASE                 0x12000000
#define FIO_BASE                0x11000000

//-----------------------------------------------------------------
// Macros:
//-----------------------------------------------------------------
#define REG8                    (volatile unsigned char*)
#define REG16                   (volatile unsigned short*)
#define REG32                   (volatile unsigned int*)

//-----------------------------------------------------------------
// Peripheral Base Addresses
//-----------------------------------------------------------------
#define UART0_BASE              (IO_BASE + 0x000)
#define TIMER_BASE              (IO_BASE + 0x100)
#define INTR_BASE               (IO_BASE + 0x200)
#define SPI_BASE                (IO_BASE + 0x300)
#define GPIO_BASE               (IO_BASE + 0x400)
#define MDIO_BASE               (IO_BASE + 0x500)
#define IICMB_BASE_ADDR         (IO_BASE + 0x600)
#define HW_MAP_BASE             (IO_BASE + 0x700)

//-----------------------------------------------------------------
// Interrupts
//-----------------------------------------------------------------
#define IRQ_UART0               0
#define IRQ_TIMER_SYSTICK       1
#define IRQ_TIMER_HIRES         2
#define IRQ_BOOT_SPI		3
#define IRQ_GPIO		4
#define IRQ_MDIO                5
#define IRQ_I2C                 6

#define IRQ_FREQMETERS          8
#define IRQ_MINIMAC_TX          9
#define IRQ_MINIMAC_RX          10

//-----------------------------------------------------------------
// Peripheral Registers
//-----------------------------------------------------------------

// UART0
#define UART0_USR           (*(REG32 (UART0_BASE + 0x4)))
#define UART0_UDR           (*(REG32 (UART0_BASE + 0x8)))

// UART1
#define UART1_USR           (*(REG32 (UART1_BASE + 0x4)))
#define UART1_UDR           (*(REG32 (UART1_BASE + 0x8)))

// TIMER
#define TIMER_VAL           (*(REG32 (TIMER_BASE + 0x0)))
#define SYS_CLK_COUNT       (*(REG32 (TIMER_BASE + 0x4)))

// IRQ
#define IRQ_MASK            (*(REG32 (INTR_BASE + 0x00)))
#define IRQ_MASK_SET        (*(REG32 (INTR_BASE + 0x00)))
#define IRQ_MASK_CLR        (*(REG32 (INTR_BASE + 0x04)))
#define IRQ_STATUS          (*(REG32 (INTR_BASE + 0x08)))

//-----------------------------------------------------------------
// pecial-Purpose Registers
//-----------------------------------------------------------------

// SR Register
#define SPR_SR                  (17)

// bits
#define SPR_SR_GIE              (1 << 2)
#define SPR_SR_ICACHE_FLUSH     (1 << 17)
#define SPR_SR_DCACHE_FLUSH     (1 << 18)

#define IOWR_8DIRECT(base, offset, val)     *((volatile unsigned long*)((base) + ((offset) << 2))) = (val)
#define IORD_8DIRECT(base, offset)          *((volatile unsigned long*)((base) + ((offset) << 2)))

#endif
