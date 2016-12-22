/****************************************************************************
 * seg7_disp.c
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


#include "seg7_display.h"

#ifdef SEG7_DISP_BASE

static const uint8_t digit2seg7_code[] = {
    sA | sB | sC | sD | sE | sF     , // 0
         sB | sC                    , // 1
    sA | sB |      sD | sE |      sG, // 2
    sA | sB | sC | sD |           sG, // 3
         sB | sC |           sF | sG, // 4
    sA |      sC | sD |      sF | sG, // 5
    sA |      sC | sD | sE | sF | sG, // 6
    sA | sB | sC                    , // 7
    sA | sB | sC | sD | sE | sF | sG, // 8
    sA | sB | sC | sD |      sF | sG, // 9
};

static const uint8_t _hex_char[] = "0123456789abcdef";

uint8_t seg7_getSegmentData(const enum Seg7Segment segment) {
    return *(REG32 (segment));
}

void seg7_setSegmentData(const enum Seg7Segment segment, const uint8_t code) {
    *(REG32 (segment)) = code;
}

uint8_t seg7_char2seg7code(const uint8_t character) {
    if ((character >= '0') && (character <= '9')) {
        return digit2seg7_code[character - '0'];
    } else if ((character >= 0) && (character <= 9)) {
        return digit2seg7_code[character];
    } else {
                // sA | sB | sC | sD | sE | sF | sG;
        switch (character) {
        case 'A':
        case 'a':
            return sA | sB | sC |      sE | sF | sG;
        case 'B':
        case 'b':
            return           sC | sD | sE | sF | sG;
        case 'C':
            return sA |           sD | sE | sF | sG;
        case 'c':
            return                sD | sE |      sG;
        case 'D':
        case 'd':
            return      sB | sC | sD | sE |      sG;
        case 'E':
        case 'e':
            return sA |           sD | sE | sF | sG;
        case 'F':
        case 'f':
            return sA |                sE | sF | sG;
        case 'G':
        case 'g':
            return digit2seg7_code[9];
        case 'H':
            return      sB | sC |      sE | sF | sG;
        case 'h':
            return           sC |      sE | sF | sG;
        case 'I':
        case 'i':
            return digit2seg7_code[1];
        case 'J':
        case 'j':
            return      sB | sC | sD               ;
        case 'L':
        case 'l':
            return                sD | sE | sF     ;
        case 'N':
        case 'n':
            return           sC |      sE |      sG;
        case 'O':
            return digit2seg7_code[0];
        case 'o':
            return           sC | sD | sE | sF     ;
        case 'P':
        case 'p':
            return sA | sB |           sE | sF | sG;
        case 'Q':
        case 'q':
            return sA | sB | sC |           sF | sG;
        case 'S':
        case 's':
            return digit2seg7_code[5];
        case 'T':
        case 't':
            return                sD | sE | sF | sG;
        case 'U':
            return      sB | sC | sD | sE | sF     ;
        case 'u':
            return           sC | sD | sE          ;
        case '_':
            return                sD               ;
        case '-':
            return                               sG;

        default:
            return 0;
        }
    }
}

void seg7_PutStr(const char *str, uint8_t size, uint8_t space_char) {
    for (uint8_t i = 0; i < SEGMENT_COUNT; ++i) {
        seg7_setSegmentData(seg7_num2Segment(i),
                       seg7_char2seg7code(i < size ? str[i] : space_char));
    }
}

bool seg7_dpGet(const enum Seg7Segment segment) {
    return (bool)(*(REG32 (segment)) & sDP);
}


void seg7_dpSet(const enum Seg7Segment segment, bool dpState) {
    uint8_t d = seg7_getSegmentData(segment);
    if (dpState) {
        d |= sDP;
    } else {
        d &= ~sDP;
    }
    seg7_setSegmentData(segment, d);
}

void seg7_printHex(uint16_t value) {
    uint8_t buf[SEGMENT_COUNT];
    for (int8_t i = SEGMENT_COUNT - 1; i >= 0; --i)
        buf[SEGMENT_COUNT - (i + 1)] = _hex_char[(value >> (4 * i)) & 0x0f];
    seg7_PutStr(buf, SEGMENT_COUNT, ' ');
}

enum Seg7Segment seg7_num2Segment(const uint8_t num) {
    return SEG7_DISP_BASE + num * 4;
}

#endif
