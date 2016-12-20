#ifndef __SERIAL_H__
#define __SERIAL_H__

//-----------------------------------------------------------------
// Defines
//-----------------------------------------------------------------
#define UART_RX_AVAIL    (1<<0)
#define UART_TX_BUSY     (1<<3)

//-----------------------------------------------------------------
// Prototypes:
//-----------------------------------------------------------------
void serial0_init (void);
int  serial0_putchar(char ch);
int  serial0_getchar(void);
int  serial0_haschar();
void serial0_putstr(char *str);

#endif // __SERIAL_H__
