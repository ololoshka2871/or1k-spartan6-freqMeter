/****************************************************************************
 * seg7_disp.h
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

#ifndef SEG7_DISP_H
#define SEG7_DISP_H

#include <stdint.h>
#include <stdbool.h>

#include "mem_map.h"

#ifdef SEG7_DISP_BASE

#define sA                  (1 << 0)
#define sB                  (1 << 1)
#define sC                  (1 << 2)
#define sD                  (1 << 3)
#define sE                  (1 << 4)
#define sF                  (1 << 5)
#define sG                  (1 << 6)
#define sDP                 (1 << 7)

enum Seg7Segment {
    SEGMENT0 = SEG7_DISP_BASE,
    SEGMENT1 = SEGMENT0 + 4,
    SEGMENT2 = SEGMENT1 + 4,
    SEGMENT3 = SEGMENT2 + 4,

    SEGMENT_COUNT = 4
};

uint8_t seg7_getSegmentData(const enum Seg7Segment segment);
void seg7_setSegmentData(const enum Seg7Segment segment, const uint8_t code);

uint8_t seg7_char2seg7code(const uint8_t character);

void seg7_PutStr(const char* str, uint8_t size, uint8_t space_char);

bool seg7_dpGet(const enum Seg7Segment segment);
void seg7_dpSet(const enum Seg7Segment segment, bool dpState);

void seg7_printHex(uint16_t value);

enum Seg7Segment seg7_num2Segment(const uint8_t num);

#endif

#endif // SEG7_DISP_H
