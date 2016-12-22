//-----------------------------------------------------------------
//                           AltOR32 
//                Alternative Lightweight OpenRisc 
//                            V2.0
//                     Ultra-Embedded.com
//                   Copyright 2011 - 2014
//
//               Email: admin@ultra-embedded.com
//
//                       License: LGPL
//-----------------------------------------------------------------
//
// Copyright (C) 2011 - 2014 Ultra-Embedded.com
//
// This source file may be used and distributed without         
// restriction provided that this copyright statement is not    
// removed from the file and that any derivative work contains  
// the original copyright notice and the associated disclaimer. 
//
// This source file is free software; you can redistribute it   
// and/or modify it under the terms of the GNU Lesser General   
// Public License as published by the Free Software Foundation; 
// either version 2.1 of the License, or (at your option) any   
// later version.
//
// This source is distributed in the hope that it will be       
// useful, but WITHOUT ANY WARRANTY; without even the implied   
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
// PURPOSE.  See the GNU Lesser General Public License for more 
// details.
//
// You should have received a copy of the GNU Lesser General    
// Public License along with this source; if not, write to the 
// Free Software Foundation, Inc., 59 Temple Place, Suite 330, 
// Boston, MA  02111-1307  USA
//-----------------------------------------------------------------

#include "gdb_hw.h"
#include "gdb.h"

#include "gdb-stub-sections.h"

#include "boot_spi.h"

//-----------------------------------------------------------------
// Defines
//-----------------------------------------------------------------
// Rx / Tx buffer max size
#define MAX_BUF_SIZE                          512

// OR1K exception reasons
#define OR32_EXC_RESET                        0
#define OR32_EXC_FAULT                        1
#define OR32_EXC_INT                          2
#define OR32_EXC_SYSCALL                      3
#define OR32_EXC_BREAKPOINT                   4
#define OR32_EXC_BUS_ERROR                    6

#define OR32_SR_STEP                          19
#define OR32_SR_DBGEN                         20

#define GDB_SIGHUP                            1
#define GDB_SIGINT                            2
#define GDB_SIGTRAP                           5
#define GDB_SIGSEGV                           11

#define REG_SP                                1
#define REG_ARG0                              3
#define REG_ARG1                              4
#define REG_ARG2                              5
#define REG_PC                                33
#define REG_SR                                34
#define REG_NUM                               35

#ifndef HEADER_W1
#define HEADER_W1                             0x03030303
#endif

#ifndef HEADER_W2
#define HEADER_W2                             0x1f1f1f1f
#endif

//-----------------------------------------------------------------
// types
//-----------------------------------------------------------------

typedef void (*pfEntry)(void) __attribute__((noreturn));

struct Hader {
    unsigned int text_start;
    unsigned int *text_dest;
    unsigned int text_len;
    unsigned int *bss_start;
    unsigned int bss_len;

    pfEntry      Entry_point;
} __attribute__((packed));

//-----------------------------------------------------------------
// Locals
//-----------------------------------------------------------------
static char       GDB_STUB_SECTION_BSS    _inbuffer[MAX_BUF_SIZE];
static char       GDB_STUB_SECTION_BSS    _outbuffer[MAX_BUF_SIZE];
       int        GDB_STUB_SECTION_BSS    _initial_trap;
static const char GDB_STUB_SECTION_RODATA _hex_char[] = "0123456789abcdef" ;

static GDB_STUB_SECTION_BSS unsigned int * (*syscall_handler)(unsigned int *registers);
static GDB_STUB_SECTION_BSS unsigned int * (*irq_handler)(unsigned int *registers);

static GDB_STUB_SECTION_BSS uint8_t cs_found;

//-----------------------------------------------------------------
// gdb_atoi_hex: Convert from hex character to integer
//-----------------------------------------------------------------
static int GDB_STUB_SECTION_TEXT gdb_atoi_hex(char ch)
{
    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    return -1;
}
//-----------------------------------------------------------------
// gdb_strcpy: String copy (avoid using any library functions)
//-----------------------------------------------------------------
static void GDB_STUB_SECTION_TEXT gdb_strcpy(char *buf, const char *s)
{
    while (*s)
        *buf++ = *s++;
    *buf = 0;
}
//-----------------------------------------------------------------
// gdb_mem2hex: Convert block of memory to ASCII
//-----------------------------------------------------------------
static char GDB_STUB_SECTION_TEXT
*gdb_mem2hex(unsigned char *mem, char *target, int count)
{
    while (count)
    {
        *target++ = _hex_char[((*mem) >> 4) & 0xf];
        *target++ = _hex_char[((*mem) >> 0) & 0xf];
        mem++;
        count--;
    }

    *target = 0;
    return target;
}
//-----------------------------------------------------------------
// gdb_hex2mem: Convert ASCII hex string to binary
//-----------------------------------------------------------------
static char GDB_STUB_SECTION_TEXT
*gdb_hex2mem(char *src, unsigned char *mem, int count)
{
    unsigned char ch;

    while (count)
    {
        ch =  gdb_atoi_hex(*src++) << 4;
        ch |= gdb_atoi_hex(*src++) << 0;

        *mem++ = ch;
        count--;
    }

    return (char *)mem;
}
//-----------------------------------------------------------------
// gdb_gethexword: Convert ASCII hex word to integer
//-----------------------------------------------------------------
static int GDB_STUB_SECTION_TEXT
gdb_gethexword(char **s, unsigned int *val)
{
    int count = 0;

    *val = 0;

    while (**s)
    {
        int hexval = gdb_atoi_hex(**s);
        if (hexval < 0)
            break;

        (*s)++;
        count++;

        *val = (*val << 4) | (hexval & 0xF);        
    }

    return count;
}
//-----------------------------------------------------------------
// gdb_send: Send GDB formatted packet ($buffer#checksum)
//-----------------------------------------------------------------
static void GDB_STUB_SECTION_TEXT gdb_send(char *buffer)
{
    unsigned char checksum;
    int index;

    do
    {
        checksum = 0;
        index    = 0;

        // Start of packet
        gdb_putchar ('$');

        // Payload (if any)
        while (buffer[index] != 0)
        {
            gdb_putchar(buffer[index]);
            checksum += (unsigned char)buffer[index++];
        }

        // Checksum
        gdb_putchar('#');
        gdb_putchar(_hex_char[(checksum >> 4) & 0xF]);
        gdb_putchar(_hex_char[(checksum >> 0) & 0xF]);
    }
    while (gdb_getchar () != '+');
}
//-----------------------------------------------------------------
// gdb_recv: Wait for valid GDB packet ($cmd#checksum).
//          'cmd' is returned
//-----------------------------------------------------------------
static char * GDB_STUB_SECTION_TEXT gdb_recv(void)
{
    int found_start = 0;
    unsigned char checksum;
    unsigned char csum_rx;
    int index;
    char ch;    

    while (1)
    {
        ch = gdb_getchar();

        // Start of packet indicator?
        if (ch == '$')
        {
            found_start = 1;
            checksum    = 0;
            csum_rx     = 0;
            index       = 0;
        }
        // Already received start of packet
        else if (found_start)
        {
            // Found start of checksum
            if (ch == '#')
                break;

            if (index < (MAX_BUF_SIZE-1))
                _inbuffer[index++] = ch;

            checksum = checksum + ch;
        }
    }
    _inbuffer[index++] = 0;
    
    // If command received without overflowing buffer
    if (index < MAX_BUF_SIZE)
    {
        // Extract / wait for checksum
        ch = gdb_getchar();
        csum_rx = gdb_atoi_hex (ch) << 4;
        ch = gdb_getchar ();
        csum_rx += gdb_atoi_hex (ch);

        // Valid checksum
        if (checksum == csum_rx)
        {
            // Sequence number?
            if (_inbuffer[2] == ':')
            {
                gdb_putchar('+');
                gdb_putchar (_inbuffer[0]);
                gdb_putchar (_inbuffer[1]);

                return &_inbuffer[3];
            }
            else
            {
                // Simple Ack
                gdb_putchar('+');

                return &_inbuffer[0];
            }            
        }
        else
        {
            _inbuffer[0] = 0;

            return &_inbuffer[0];
        }
    }
    else
    {
        _inbuffer[0] = 0;

        return &_inbuffer[0];
    }
}

//-----------------------------------------------------------------
// syscall_handler
//-----------------------------------------------------------------
unsigned int * GDB_STUB_SECTION_TEXT
gdb_syscall(unsigned int *registers)
{
    unsigned int len;
    char *str;

    // Get l.sys opcode
    //unsigned int opcode = *((unsigned int*)(registers[REG_PC] - 4));
    unsigned int opcode = registers[REG_PC];
    opcode -= 4;
    opcode = *((unsigned int*)opcode);
    unsigned int sys_num = opcode & 0xFFFF;

    char * ptr = _outbuffer;
    switch (sys_num)
    {
      //---------------------------------------------------
      // l.sys 1 -> putchar(r3)
      //---------------------------------------------------
      case 1:
          *ptr++ = 'O';
          *ptr++ = _hex_char[(registers[REG_ARG0] >> 4) & 0xf];
          *ptr++ = _hex_char[registers[REG_ARG0] & 0xf];
          *ptr++ = 0;
          gdb_send (_outbuffer);

          return registers;
      //---------------------------------------------------
      // l.sys 2 -> putstr(r3)
      //---------------------------------------------------
      case 2:
          // Pointer to string
          str = (char*)registers[REG_ARG0];
          len = 0;

          *ptr++ = 'O';
          while (*str && (len < ((sizeof(_outbuffer)-2)/2)))
          {
              *ptr++ = _hex_char[((*str) >> 4) & 0xf];
              *ptr++ = _hex_char[((*str) >> 0) & 0xf];
              str++;
          }
          *ptr++ = 0;
          gdb_send (_outbuffer);

          return registers;
      //---------------------------------------------------
      // l.sys 3 -> exit(r3)
      //---------------------------------------------------
      case 3: // TODO!!!!
          *ptr++ = 'W';
          *ptr++ = _hex_char[(registers[REG_ARG0] >> 4) & 0xf];
          *ptr++ = _hex_char[registers[REG_ARG0] & 0xf];
          *ptr++ = 0;
          gdb_send (_outbuffer);

          return registers;
      //---------------------------------------------------
      // l.sys 4 -> set syscall_handler = r3
      //---------------------------------------------------
      case 4:
      {
          void* oldhandler = syscall_handler;
          syscall_handler = (void*)registers[REG_ARG0];
          registers[REG_ARG0] = (unsigned int)oldhandler;
          return registers;
      }
      //---------------------------------------------------
      // l.sys 5 -> set irq_handler = r3
      //---------------------------------------------------
      case 5:
      {
          void* oldhandler = syscall_handler;
          irq_handler = (void*)registers[REG_ARG0];
          registers[REG_ARG0] = (unsigned int)oldhandler;
          return registers;
      }
      //---------------------------------------------------
      // l.sys 6 -> read boot flash
      // R3 - addr to read from
      // R4 - destination buffer
      // R5 - size to read
      // Return R3 == R5 if success or 0 if error
      //---------------------------------------------------
      case 6:
          if (!cs_found) {
              registers[REG_ARG0] = 0;
          } else {
            uint32_t read_addr = registers[REG_ARG0];
            unsigned char* destbuf = (unsigned char*)registers[REG_ARG1];
            uint32_t size_to_read = registers[REG_ARG2];
            spi_flash_read(cs_found, read_addr, destbuf, size_to_read);
            registers[REG_ARG0] = size_to_read;
          }

          return registers;
      //---------------------------------------------------
      // l.sys 7 -> reboot
      // No input
      // Return void
      //---------------------------------------------------
      case 7:
        asm volatile(
        /* Change SR to User Mode */
        "l.mfspr r3,r0,17\n" /* copy SR to R3 */
        "l.andi r3,r3,0xFFFE\n" /* change bit 0 to 0 */
        "l.mtspr r0,r3,17\n" /* move back to SR */

        /* Jump to main */
        "l.movhi r2,hi(vector_reset)\n"
        "l.ori r2,r2,lo(vector_reset)\n"
        "l.jr r2\n"
        "l.nop\n");

      //---------------------------------------------------
      // Default: User syscall
      //---------------------------------------------------
      default:
          if (syscall_handler)
              return syscall_handler(registers);
          // Not supported
          else
          {
              registers[REG_ARG0] = 0;
              return registers;
          }
    }
}

//-----------------------------------------------------------------
// interrupt_handler
//-----------------------------------------------------------------
unsigned int * GDB_STUB_SECTION_TEXT
interrupt_handler(unsigned int *registers)
{
    if (irq_handler)
        return irq_handler(registers);
    else
        return registers;
}

//-----------------------------------------------------------------
// gdb_exception
//-----------------------------------------------------------------
unsigned int * GDB_STUB_SECTION_TEXT
gdb_exception(unsigned int *registers, unsigned int reason)
{
    int sig_val;
    unsigned int len;
    unsigned int val;
    unsigned int regnum;
    char *ptr;    
    int flush_caches = 0;
    
    switch (reason)
    {
    case OR32_EXC_INT:
      sig_val = GDB_SIGINT;
      break;
    case OR32_EXC_BREAKPOINT:
      sig_val = GDB_SIGTRAP;
      break;
    case OR32_EXC_FAULT:
      sig_val = GDB_SIGSEGV;
      break;
    default:
      sig_val = GDB_SIGHUP;
      break;
    }

    // Exception due external interrupt
    if (reason == OR32_EXC_INT)
    {
        interrupt_handler(registers);
    }
    // Exception due to syscall instruction
    else if (reason == OR32_EXC_SYSCALL)
    {
        return gdb_syscall(registers);
    }

    // Make sure debug enabled on return to user program
    registers[REG_SR] |= (1 << OR32_SR_DBGEN);

    // Send status response (signal type, PC & SP)
    if (!_initial_trap)
    {
        ptr = _outbuffer;
        *ptr++ = 'T';
        *ptr++ = _hex_char[(sig_val >> 4) & 0xf];
        *ptr++ = _hex_char[sig_val & 0xf];
        *ptr++ = _hex_char[(REG_PC >> 4) & 0xf];
        *ptr++ = _hex_char[REG_PC & 0xf];
        *ptr++ = ':';
        ptr = gdb_mem2hex ((unsigned char *)&registers[REG_PC], ptr, 4);
        *ptr++ = ';';
        *ptr++ = _hex_char[(REG_SP >> 4) & 0xf];
        *ptr++ = _hex_char[REG_SP & 0xf];
        *ptr++ = ':';
        ptr = gdb_mem2hex ((unsigned char *)&registers[REG_SP], ptr, 4);
        *ptr++ = ';';
        *ptr++ = 0;  
        gdb_send (_outbuffer);
    }
    // Initial trap (jump to GDB stub)
    else
    {
        sig_val = GDB_SIGHUP;
        _initial_trap = 0;
    }

    while (1)
    {
        // Wait for request from GDB
        ptr = gdb_recv();
        
        _outbuffer[0] = 0;
        switch (*ptr++)
        {
        //---------------------------------------------------
        // ? - Return signal value
        //---------------------------------------------------
        case '?':
            _outbuffer[0] = 'S';
            _outbuffer[1] = _hex_char[sig_val >> 4];
            _outbuffer[2] = _hex_char[sig_val & 0xf];
            _outbuffer[3] = 0;
            break;        
        //---------------------------------------------------
        // g - Return all registers
        //---------------------------------------------------
        case 'g':
            ptr = gdb_mem2hex ((unsigned char *)registers, _outbuffer, REG_NUM * 4); 
            break;
        //---------------------------------------------------
        // G - Set all registers
        //---------------------------------------------------
        case 'G':
            gdb_hex2mem (ptr, (unsigned char *)registers, REG_NUM * 4);
            gdb_strcpy (_outbuffer, "OK");

            // Make sure debug enabled on return to user program
            registers[REG_SR] |= (1 << OR32_SR_DBGEN);
            break;
        //---------------------------------------------------
        // p - Return a single register
        //---------------------------------------------------
        case 'p':
            if (gdb_gethexword (&ptr, &val) && (val < REG_NUM))
                ptr = gdb_mem2hex ((unsigned char *)&registers[val], _outbuffer, 4);
            else 
                gdb_strcpy (_outbuffer, "E22");
            break;
        //---------------------------------------------------
        // P - Set a single register
        //---------------------------------------------------
        case 'P':
            // Get register number
            if (gdb_gethexword (&ptr, &regnum) && (*ptr++ == '=') && (regnum < REG_NUM))
            {
                // Get value to set register to
                gdb_gethexword(&ptr, &val);
                registers[regnum] = val;

                // If SR, make sure debug enabled on return to user program
                if (regnum == REG_SR)
                    registers[regnum] |= (1 << OR32_SR_DBGEN);

                gdb_strcpy (_outbuffer, "OK");
            }
            else
                gdb_strcpy (_outbuffer, "E22");
            break;
        //---------------------------------------------------
        // m - Read a block of memory
        //---------------------------------------------------
        case 'm':
            if (gdb_gethexword (&ptr, &val) && (*ptr++ == ',') &&
                gdb_gethexword (&ptr, &len) && (len < ((sizeof(_outbuffer)-1)/2)))
            {
                if (!gdb_mem2hex((unsigned char *)val, _outbuffer, len))
                    gdb_strcpy (_outbuffer, "E14");
            }
            else
                gdb_strcpy (_outbuffer,"E22");
            break;
        //---------------------------------------------------
        // M - Write a block of memory
        //---------------------------------------------------
        case 'M':
            if (gdb_gethexword (&ptr, &val) && (*ptr++ == ',') && 
                gdb_gethexword (&ptr, &len) && (*ptr++ == ':'))
            {
                if (gdb_hex2mem(ptr, (unsigned char *)val, len))
                    gdb_strcpy (_outbuffer, "OK");
                else
                    gdb_strcpy (_outbuffer, "E14");

                flush_caches = 1;
            }
            else
                gdb_strcpy (_outbuffer, "E22");
            break;
        //---------------------------------------------------
        // c - Continue from address (or last PC)
        //---------------------------------------------------
        case 'c':
            // Optional PC
            if (gdb_gethexword (&ptr, &val))
                registers[REG_PC] = val;

            if (flush_caches)
                gdb_flush_cache();

            return registers;
        //---------------------------------------------------
        // s - Step from address (or last PC)
        //---------------------------------------------------
        case 's':
            // Optional PC
            if (gdb_gethexword (&ptr, &val))
                registers[REG_PC] = val;

            // Set step in ESR and make sure debug is enabled
            registers[REG_SR] |= (1 << OR32_SR_STEP);
            registers[REG_SR] |= (1 << OR32_SR_DBGEN);
          
            if (flush_caches)
                gdb_flush_cache();

            return registers;
        }

        // Send response to GDB host
        gdb_send (_outbuffer);
    }
}

//-----------------------------------------------------------------
// FATAL_no_bootable_code_found
//-----------------------------------------------------------------
static void GDB_STUB_SECTION_TEXT FATAL_no_bootable_code_found(int code) {
    gdb_putstr("Boot error: ");
    gdb_putchar('0' + code);
    gdb_putchar('\n');
    gdb_putchar('\r');
    while(1);
}

//-----------------------------------------------------------------
// Boot_status
//-----------------------------------------------------------------
static void GDB_STUB_SECTION_TEXT Boot_status(int code) {
    gdb_putchar('A' + code);
    gdb_putchar('\n');
    gdb_putchar('\r');
}

//-----------------------------------------------------------------
// try_load
//-----------------------------------------------------------------
void GDB_STUB_SECTION_TEXT try_load(void) {
    pfEntry user_code_entry = 0;
    {
        uint32_t buf;

        //Probing flash
        cs_found = spi_probe_flash(1, 2);

        if (!cs_found)
            FATAL_no_bootable_code_found(0);

        // Start searching for hader
        Boot_status(0);
        for (uint32_t addr = 0; addr < ((1ul << 25) - 1);
            addr += sizeof(uint32_t)) {
            spi_flash_read(cs_found, addr, (unsigned char*)&buf,
                           sizeof(uint32_t));
            if (buf == SYSTEM_HEADER_W1) {
                spi_flash_read(cs_found, addr + sizeof(uint32_t),
                               (unsigned char*)&buf, sizeof(uint32_t));
                if (buf == SYSTEM_HEADER_W2) {
                    struct Hader user_code_hader;
                    spi_flash_read(cs_found, addr + 2 * sizeof(uint32_t),
                                   (unsigned char*)&user_code_hader,
                                   sizeof(struct Hader));
                    Boot_status(1);
                    if(user_code_hader.text_start != addr ||
                        (void*)user_code_hader.text_dest < (void*)try_load ||
                        (void*)user_code_hader.bss_start < (void*)try_load)
                    {
                        addr += sizeof(uint32_t);
                        continue;
                    }
                    // Firmware found, loading
                    spi_flash_read(cs_found, user_code_hader.text_start,
                                   (unsigned char*)user_code_hader.text_dest,
                                   user_code_hader.text_len);
                    Boot_status(2);
                    // clear bss
                    if (user_code_hader.bss_len) {
                        while (user_code_hader.bss_len -= 4) {
                            *user_code_hader.bss_start = 0;
                            user_code_hader.bss_start++;
                        }
                    }
                    user_code_entry = user_code_hader.Entry_point;
                    break;
                }
            }
        }
    }
    if (user_code_entry) {
        // Entering user application..
        Boot_status(3);
        user_code_entry();
    } else
        FATAL_no_bootable_code_found(1);
}

