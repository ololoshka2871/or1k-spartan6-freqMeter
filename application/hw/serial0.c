#include "mem_map.h"
#include "serial.h"

//-------------------------------------------------------------
// serial_init: 
//-------------------------------------------------------------
void serial0_init (void)
{      
    // Not required
}
//-------------------------------------------------------------
// serial_putchar: Write character to UART Tx buffer
//-------------------------------------------------------------
int serial0_putchar(char ch)
{   
    if (ch == '\n')
        serial1_putchar('\r');
    
    // Print in simulator via l.nop instruction
    {
        register char  t1 asm ("r3") = ch;
        asm volatile ("\tl.nop\t%0" : : "K" (0x0004), "r" (t1));
    }

    // Write to Tx buffer
    UART0_UDR = ch;

    // Wait for Tx to complete
    while (UART0_USR & UART_TX_BUSY);

    return 0;
}
//-------------------------------------------------------------
// serial_getchar: Read character from UART Rx buffer
//-------------------------------------------------------------
int serial0_getchar (void)
{
    if (serial1_haschar())
        return UART0_UDR;
    else
        return -1;
}
//-------------------------------------------------------------
// serial_haschar: Is a character waiting in Rx buffer
//-------------------------------------------------------------
int serial0_haschar()
{
    return (UART0_USR & UART_RX_AVAIL);
}
//-------------------------------------------------------------
// serial_putstr: Send a string to UART
//-------------------------------------------------------------
void serial0_putstr(char *str)
{
    while (*str)
        serial1_putchar(*str++);
}
