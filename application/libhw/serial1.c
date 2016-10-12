#include "mem_map.h"
#include "serial.h"

//-------------------------------------------------------------
// serial_init: 
//-------------------------------------------------------------
void serial1_init (void)
{      
    // Not required
}
//-------------------------------------------------------------
// serial_putchar: Write character to UART Tx buffer
//-------------------------------------------------------------
int serial1_putchar(char ch)
{   
    // Write to Tx buffer
    UART1_UDR = ch;

    // Wait for Tx to complete
    while (UART1_USR & UART_TX_BUSY);

    return 0;
}
//-------------------------------------------------------------
// serial_getchar: Read character from UART Rx buffer
//-------------------------------------------------------------
int serial1_getchar (void)
{
    if (serial1_haschar())
        return UART1_UDR;
    else
        return -1;
}
//-------------------------------------------------------------
// serial_haschar: Is a character waiting in Rx buffer
//-------------------------------------------------------------
int serial1_haschar()
{
    return (UART1_USR & UART_RX_AVAIL);
}
//-------------------------------------------------------------
// serial_putstr: Send a string to UART
//-------------------------------------------------------------
void serial1_putstr(char *str)
{
    while (*str)
        serial1_putchar(*str++);
}
