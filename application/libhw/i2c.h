#ifndef I2C_H
#define I2C_H

#include "iicmb.h"
#include "bb_i2c.h"

#if ENABLE_I2C
// hw i2c
#define i2c_init                            iicmb_init
#define i2c_disable                         iicmb_disable

#define i2c_cmd_wait(n)                     iicmb_cmd_wait(n)
#define i2c_cmd_write(n)                    iicmb_cmd_write(n)
#define i2c_cmd_read_ack(n)                 iicmb_cmd_read_ack(n)
#define i2c_cmd_read_nak(n)                 iicmb_cmd_read_nak(n)
#define i2c_cmd_start                       iicmb_cmd_start
#define i2c_cmd_stop                        iicmb_cmd_stop
#define i2c_cmd_set_bus(n)                  iicmb_cmd_set_bus(n)

#elif ENABLE_GPIO
// bitbang i2c
#define i2c_init                            bb_i2c_init
#define i2c_disable                         bb_i2c_disable

#define i2c_cmd_wait(n)                     bb_i2c_cmd_wait(n)
#define i2c_cmd_write(n)                    bb_i2c_cmd_write(n)
#define i2c_cmd_read_ack(n)                 bb_i2c_cmd_read_ack(n)
#define i2c_cmd_read_nak(n)                 bb_i2c_cmd_read_nak(n)
#define i2c_cmd_start                       bb_i2c_cmd_start
#define i2c_cmd_stop                        bb_i2c_cmd_stop
#define i2c_cmd_set_bus(n)                  bb_i2c_cmd_set_bus(n)

#else
#warning "No i2c implementation avalable, all calls will have no effect!"
#define I2C_DISABLED

#define i2c_init()
#define i2c_disable()

#define i2c_cmd_wait(n)                     rsp_done
#define i2c_cmd_write(n)                    rsp_done
#define i2c_cmd_read_ack(n)                 rsp_done
#define i2c_cmd_read_nak(n)                 rsp_done
#define i2c_cmd_start()                     rsp_done
#define i2c_cmd_stop()                      rsp_done
#define i2c_cmd_set_bus(n)                  rsp_done
#endif

/* High-level operations: ****************************************************/

/* Read a single byte
 * Parameters:
 *    unsigned char    sa   -- I2C Slave address (7-bit)
 *    unsigned char    a    -- Byte address
 *    unsigned char *  d    -- Pointer to a storage for received data
 * Returns:
 *    rsp_tt                -- Response
 */
rsp_tt i2c_read_bus(unsigned char sa, unsigned char a, unsigned char * d);

/* Read several bytes
 * Parameters:
 *    unsigned char    sa   -- I2C Slave address (7-bit)
 *    unsigned char    a    -- Byte address
 *    unsigned char *  d    -- Pointer to a storage for received data
 *    int              n    -- Number of bytes to read
 * Returns:
 *    rsp_tt                -- Response
 */
rsp_tt i2c_read_bus_mul(unsigned char sa, unsigned char a, unsigned char * d, int n);


/* Write a single byte
 * Parameters:
 *    unsigned char    sa   -- I2C Slave address (7-bit)
 *    unsigned char    a    -- Byte address
 *    unsigned char    d    -- Data byte to write
 * Returns:
 *    rsp_tt                -- Response
 */
rsp_tt i2c_write_bus(unsigned char sa, unsigned char a, unsigned char d);


/* Write several bytes
 * Parameters:
 *    unsigned char    sa   -- I2C Slave address (7-bit)
 *    unsigned char    a    -- Byte address
 *    unsigned char *  d    -- Pointer to a storage with data to write
 *    int              n    -- Number of bytes to write
 * Returns:
 *    rsp_tt                -- Response
 */
rsp_tt i2c_write_bus_mul(unsigned char sa, unsigned char a, unsigned char * d, int n);

#endif // I2C_H
