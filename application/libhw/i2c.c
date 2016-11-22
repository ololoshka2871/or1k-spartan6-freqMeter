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


#include "i2c.h"

rsp_tt i2c_read_bus(unsigned char sa, unsigned char a, unsigned char *d)
{
    rsp_tt ret;

    /* Start condition */
    ret = i2c_cmd_start();
    if (ret != rsp_done) return ret;

    do {
      /* Write slave address and write bit */
      ret = i2c_cmd_write((sa << 1) | 0x00u);
      if (ret != rsp_done) break;
      /* Write byte address */
      ret = i2c_cmd_write(a);
      if (ret != rsp_done) break;
      /* Repeated start */
      ret = i2c_cmd_start();
      if (ret != rsp_done) return ret;
      /* Write slave address and read bit */
      ret = i2c_cmd_write((sa << 1) | 0x01u);
      if (ret != rsp_done) break;
      /* Read byte of data with not-acknowledge */
      ret = i2c_cmd_read_nak(d);
      if (ret != rsp_done) return ret;
    } while (0);

    /* Stop condition */
    (void)i2c_cmd_stop();

    return ret;
}

rsp_tt i2c_read_bus_mul(unsigned char sa, unsigned char a, unsigned char *d, int n)
{
    rsp_tt ret;
    int i;

    /* Start condition */
    ret = i2c_cmd_start();
    if (ret != rsp_done) return ret;

    do {
      /* Write slave address and write bit */
      ret = i2c_cmd_write((sa << 1) | 0x00u);
      if (ret != rsp_done) break;
      /* Write byte address */
      ret = i2c_cmd_write(a);
      if (ret != rsp_done) break;
      /* Repeated start */
      ret = i2c_cmd_start();
      if (ret != rsp_done) return ret;
      /* Write slave address and read bit */
      ret = i2c_cmd_write((sa << 1) | 0x01u);
      if (ret != rsp_done) break;
      for (i = 0; i < (n - 1); i++)
      {
        /* Read byte of data with acknowledge */
        ret = i2c_cmd_read_ack(d + i);
        if (ret != rsp_done) return ret;
      }
      /* Read byte of data with not-acknowledge */
      ret = i2c_cmd_read_nak(d + i);
      if (ret != rsp_done) return ret;
    } while (0);

    /* Stop condition */
    (void)i2c_cmd_stop();

    return ret;
}

rsp_tt i2c_write_bus(unsigned char sa, unsigned char a, unsigned char d)
{
    rsp_tt ret;

    /* Start condition */
    ret = i2c_cmd_start();
    if (ret != rsp_done) return ret;

    do {
      /* Write slave address and write bit */
      ret = i2c_cmd_write((sa << 1) | 0x00000000);
      if (ret != rsp_done) break;
      /* Write byte address */
      ret = i2c_cmd_write(a);
      if (ret != rsp_done) break;
      /* Write byte of data */
      ret = i2c_cmd_write(d);
      if (ret != rsp_done) break;
    } while (0);

    /* Stop condition */
    (void)i2c_cmd_stop();

    return ret;
}

rsp_tt i2c_write_bus_mul(unsigned char sa, unsigned char a, unsigned char *d, int n)
{
    rsp_tt ret;
    int i;

    /* Start condition */
    ret = i2c_cmd_start();
    if (ret != rsp_done) return ret;

    do {
      /* Write slave address and write bit */
      ret = i2c_cmd_write((sa << 1) | 0x00000000);
      if (ret != rsp_done) break;
      /* Write byte address */
      ret = i2c_cmd_write(a);
      if (ret != rsp_done) break;
      for (i = 0; i < n; i++)
      {
        /* Write byte of data */
        ret = i2c_cmd_write(*(d + i));
        if (ret != rsp_done) break;
      }
    } while (0);

    /* Stop condition */
    (void)i2c_cmd_stop();

    return ret;
}
