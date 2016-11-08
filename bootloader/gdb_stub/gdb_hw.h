#ifndef __GDB_HW_H__
#define __GDB_HW_H__

//-----------------------------------------------------------------
// Prototypes:
//-----------------------------------------------------------------
void gdb_putchar(char c);
void gdb_putstr(const char *str);
int  gdb_getchar(void);
void gdb_flush_cache(void);

unsigned long mfspr(unsigned long spr);
void mtspr(unsigned long spr, unsigned long value);

#endif
