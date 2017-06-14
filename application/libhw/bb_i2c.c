/****************************************************************************
 *
 *   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
 *   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
 *   Based on iicmb: http://opencores.org/project,iicmb
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <assert.h>

#include "GPIO.h"
#ifndef SIM
#include "timer.h"
#endif

#include "i2c.h"

#include "bb_i2c.h"

#if !ENABLE_I2C && !defined(I2C_DISABLED)

#ifndef BB_I2C_PORT
#warning "BB_I2C_PORT not defined, assuming PORTA"
#define BB_I2C_PORT             GPIO_PORTA
#endif

#ifndef BB_I2C_SDA_PIN
#warning "BB_I2C_SDA_PIN not defined, assuming 0"
#define BB_I2C_SDA_PIN_MASK     (1 << 0)
#else
#define BB_I2C_SDA_PIN_MASK     (1 << (BB_I2C_SDA_PIN))
#endif

#ifndef BB_I2C_SCL_PIN
#warning "BB_I2C_SCL_PIN not defined, assuming 1"
#define BB_I2C_SCL_PIN_MASK     (1 << 1)
#else
#define BB_I2C_SCL_PIN_MASK     (1 << (BB_I2C_SCL_PIN))
#endif

#ifndef BB_I2C_BAUD
#warning "BB_I2C_BAUD not defined, assuming 100000"
#define BB_I2C_BAUD             100000
#endif


#define SDA_0()                 gpio_port_set_dir(i2c_gpio_port, BB_I2C_SDA_PIN_MASK, 0)
#define SDA_1()                 gpio_port_set_dir(i2c_gpio_port, 0, BB_I2C_SDA_PIN_MASK)

#define SDA_STATUS()            (gpio_port_get_val(i2c_gpio_port) & BB_I2C_SDA_PIN_MASK)

#define SCL_0()                 gpio_port_set_dir(i2c_gpio_port, BB_I2C_SCL_PIN_MASK, 0)
#define SCL_1()                 gpio_port_set_dir(i2c_gpio_port, 0, BB_I2C_SCL_PIN_MASK)

#define SCL_STATUS()            (gpio_port_get_val(i2c_gpio_port) & BB_I2C_SCL_PIN_MASK)

static GPIO i2c_gpio_port = NULL;

static void bb_i2c_delay_05T() {
#ifndef SIM
    hires_timer_sleep(F_CPU / (14 /* 7@50MHz experimental*/ * BB_I2C_BAUD / 2));
#endif
}

void bb_i2c_init(void) {
    i2c_gpio_port = (GPIO)BB_I2C_PORT;

    gpio_port_set_dir(i2c_gpio_port, 0, BB_I2C_SDA_PIN_MASK | BB_I2C_SCL_PIN_MASK);
    gpio_port_set_val(i2c_gpio_port, 0, BB_I2C_SDA_PIN_MASK | BB_I2C_SCL_PIN_MASK); // 0 if out
}

void bb_i2c_disable(void) {
    gpio_port_set_dir(i2c_gpio_port, 0, BB_I2C_SDA_PIN_MASK | BB_I2C_SCL_PIN_MASK);
}

rsp_tt bb_i2c_cmd_wait(unsigned char n) { return rsp_done; }

rsp_tt bb_i2c_cmd_write(unsigned char n) {
    //shift out bits
    for(uint8_t i = 0; i < 8; ++i) {
        //pull SCL low
        SCL_0();
        //check bit
        if (n & (1 << 7)) {
            //float SDA
            SDA_1();
        } else {
            //pull SDA low
            SDA_0();
        }
        //shift
        n<<=1;
        //wait for 1/2 clock
        bb_i2c_delay_05T();
        //float SCL
        SCL_1();
        //wait for 1/2 clock
        bb_i2c_delay_05T();
    }
    //check ack bit
    //pull SCL low
    SCL_0();
    //float SDA
    SDA_1();
    //wait for 1/2 clock
    bb_i2c_delay_05T();
    //float SCL
    SCL_1();
    //wait for 1/2 clock
    bb_i2c_delay_05T();
    //sample SDA
    n = SDA_STATUS() ? rsp_nak : rsp_done;
    //pull SCL low
    SCL_0();
    //return sampled value
    return (rsp_tt)n;
}

static unsigned char bb_i2c_cmd_read_com() {
    unsigned char val = 0;

    //shift out bits
    for(uint8_t i = 0; i < 8; ++i){
        //pull SCL low
        SCL_0();
        //wait for 1/2 clock
        bb_i2c_delay_05T();
        //float SCL
        SCL_1();
        //wait for 1/2 clock
        bb_i2c_delay_05T();
        //shift value to make room
        val<<=1;
        //sample data
        if(SDA_STATUS()){
            val |= 1;
        }
    }

    //pull SCL low
    SCL_0();

    return val;
}

static void finalise_read_com() {
    //wait for 1/2 clock
    bb_i2c_delay_05T();
    //float SCL
    SCL_1();
    //wait for 1/2 clock
    bb_i2c_delay_05T();
    //pull SCL low
    SCL_0();
    //float SDA
    SDA_1();
}

rsp_tt bb_i2c_cmd_read_ack(unsigned char * n) {
    *n = bb_i2c_cmd_read_com();

    //pull SDA low for ACK
    SDA_0();

    finalise_read_com();
    return rsp_done;
}

rsp_tt bb_i2c_cmd_read_nak(unsigned char * n) {
    *n = bb_i2c_cmd_read_com();

    //float SDA for NACK
    SDA_1();

    finalise_read_com();
    return rsp_done;
}

rsp_tt bb_i2c_cmd_start(void) {
    if (!SDA_STATUS())
        return rsp_arb_lost;

    if (!SCL_STATUS()) {
        bb_i2c_delay_05T();
        SCL_1();
    }
    //wait for 1/2 clock first
    bb_i2c_delay_05T();
    //pull SDA low
    SDA_0();
    //wait for 1/2 clock for end of start
    bb_i2c_delay_05T();

    return rsp_done;
}

rsp_tt bb_i2c_cmd_stop(void) {
    //pull SDA low
    SDA_0();
    //wait for 1/2 clock for end of start
    bb_i2c_delay_05T();
    //float SCL
    SCL_1();
    //wait for 1/2 clock
    bb_i2c_delay_05T();
    //float SDA
    SDA_1();
    //wait for 1/2 clock
    bb_i2c_delay_05T();

    return rsp_done;
}

rsp_tt bb_i2c_cmd_set_bus(unsigned char n) {
    return rsp_done;
}

#endif
