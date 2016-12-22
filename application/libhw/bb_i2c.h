#ifndef BB_I2C_H
#define BB_I2C_H

#include "GPIO.h"

#include "iicmb.h"

void bb_i2c_init(void);
void bb_i2c_disable(void);

rsp_tt bb_i2c_cmd_wait(unsigned char n);
rsp_tt bb_i2c_cmd_write(unsigned char n);
rsp_tt bb_i2c_cmd_read_ack(unsigned char * n);
rsp_tt bb_i2c_cmd_read_nak(unsigned char * n);
rsp_tt bb_i2c_cmd_start(void);
rsp_tt bb_i2c_cmd_stop(void);
rsp_tt bb_i2c_cmd_set_bus(unsigned char n);

#endif // BB_I2C_H
