/****************************************************************************
 * GPIO.c
 *
 *   Copyright (C) 2016 Shilo_XyZ_. All rights reserved.
 *   Author:  Shilo_XyZ_ <Shilo_XyZ_<at>mail.ru>
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

#include "GPIO.h"

struct sGPIO {
	uint32_t IN; // input
	uint32_t OUT; // output
	uint32_t OE; // direction
	uint32_t INTE; // interrupt mask
	uint32_t PTRIG; // manual trigger interrupt
	uint32_t AUX;  // unimplemented in hw
	uint32_t CTRL; // control
	uint32_t INTS; // interrupt flags
	uint32_t ECLK; // unimplemented in hw
	uint32_t NEC; // unimplemented in hw
};

GPIO gpio_port_init(enum GPIO_PORTS port, uint32_t direction) {
	volatile struct sGPIO* result = (volatile struct sGPIO*)port; // base address
	result->OUT = 0;
	result->OE = direction;
	result->CTRL &= ~GPIO_CTRL_INTE;

	return (GPIO)result;
}


void gpio_port_set_dir_all(GPIO gpio, uint32_t direction) {
	volatile struct sGPIO* p = (volatile struct sGPIO*)gpio;
	p->OE = direction;
}


void gpio_port_set_all(GPIO gpio, uint32_t val) {
	volatile struct sGPIO* p = (volatile struct sGPIO*)gpio;
	p->OUT = val;
}


void gpio_port_set_dir(GPIO gpio, uint32_t set_out_mask, uint32_t set_in_mask) {
    uint32_t dir = gpio_port_get_dir(gpio);
    dir &= ~set_in_mask;
    dir |= set_out_mask;
    gpio_port_set_dir_all(gpio, dir);
}


void gpio_port_set_val(GPIO gpio, uint32_t set_mask, uint32_t unset_mask) {
	uint32_t v = gpio_port_get_val(gpio);
	v &= ~unset_mask;
	v |= set_mask;
	gpio_port_set_all(gpio, v);
}


uint32_t gpio_port_get_val(GPIO gpio) {
	return ((volatile struct sGPIO*)gpio)->IN;
}


uint32_t gpio_port_get_dir(GPIO gpio) {
    volatile struct sGPIO* p = (volatile struct sGPIO*)gpio;
    return p->OE;
}
