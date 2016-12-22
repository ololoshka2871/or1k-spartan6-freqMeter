/****************************************************************************
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

#include <string.h>
#include <assert.h>
#include <stdbool.h>

#include "syscall.h"
#include "crc32.h"
#include "mem_managment.h"

#include "rodata.h"

#ifndef RODATA_BLOB_START
#error "RODATA_BLOB_START not defined, where to find site pages?"
#endif

#ifndef RODATA_FILENAME_LEN
#warning "RODATA_FILENAME_LEN not defined, assuming 21"
#define RODATA_FILENAME_LEN  21
#endif

#ifndef RODATA_FILEEXT_LEN
#warning "RODATA_FILEEXT_LEN not defined, assuming 21"
#define RODATA_FILEEXT_LEN  21
#endif


struct rodata_file {
    char fileExt[RODATA_FILEEXT_LEN];
    char fileName[RODATA_FILENAME_LEN];
    uint32_t startOffset;
    uint32_t size;
} __attribute__((packed));

struct rodata_hader {
    uint32_t rodata_table_size;
    uint32_t rodata_table_crc;
    char reserved[16 - 2 * sizeof(uint32_t)];
} __attribute__((packed));

struct rodata_fs {
    struct rodata_hader hader;
    struct rodata_file records[20 /*invalid value, only for debug frendly*/];
} __attribute__((packed));

static struct rodata_fs* prodata_fs = NULL;

#define CACHING_SUPPORT

#ifdef CACHING_SUPPORT
#ifndef RODATA_CACHE_SIZE
#warning "RODATA_CACHE_SIZE not defined, assuming 128"
#define RODATA_CACHE_SIZE  128
#endif

static uint8_t rodata_cache[RODATA_CACHE_SIZE];
static uint32_t rodata_cached_start = 0;
#endif

#define FILES_IN_TABLE()                ((prodata_fs->hader.rodata_table_size - sizeof(struct rodata_hader))/sizeof(struct rodata_file))
#define FILE_HADER(descriptor)          (prodata_fs->records[descriptor])
#define FILE_CONTENT_ADDR(descriptor)   (prodata_fs->records[descriptor].startOffset + RODATA_BLOB_START)

static void rodata_assert(bool condition) {
    assert(condition);
}

#ifdef CACHING_SUPPORT
static void read_from_cache(uint8_t *buf, uint32_t start, uint32_t size) {
    while(size) {
        uint32_t segment_offset = start % RODATA_CACHE_SIZE;
        uint32_t requred_segment = start - segment_offset;
        if (requred_segment != rodata_cached_start) {
            // cache new segment
            rodata_cached_start = requred_segment;
            read_boot_flash(rodata_cached_start, rodata_cache, RODATA_CACHE_SIZE);
        }

        uint32_t sizetoread = RODATA_CACHE_SIZE - segment_offset;
        if (sizetoread > size)
            sizetoread = size;

        if (size == 1)
            *buf = rodata_cache[segment_offset];
        else
            memcpy(buf, &rodata_cache[segment_offset], sizetoread);

        size  -= sizetoread;
        start += sizetoread;
        buf   += sizetoread;
    }
}
#endif

static bool rodata_init() {
    struct rodata_hader h;
    read_boot_flash(RODATA_BLOB_START, (uint8_t*)&h, sizeof(struct rodata_hader));

    if (h.rodata_table_size == ~0)
        return false; // size invalid! can't init table

    uint32_t prodata_fs_size = h.rodata_table_size;

    struct rodata_fs* table = malloc_sys(prodata_fs_size);
    if (!table)
        return false; // E_NOMEM
    read_boot_flash(RODATA_BLOB_START, (uint8_t*)table, prodata_fs_size);

    uint32_t origin_crc = table->hader.rodata_table_crc;
    table->hader.rodata_table_crc = 0;
    uint32_t calculed_crc = crc32(table, prodata_fs_size);

    if (origin_crc != calculed_crc)
        return false; // table content invalid

    table->hader.rodata_table_crc = origin_crc;
    prodata_fs = table; // init ok
    return true;
}

static int8_t is_file_mach(char *name, char *ext, uint32_t list_index) {
    char* liName = FILE_HADER(list_index).fileName;
    char* liExt  = FILE_HADER(list_index).fileExt;
    uint32_t i;

    for (i = 0; i < RODATA_FILEEXT_LEN; ++i) {
        if (!ext[i])
            return liExt[i] != 0;
        if (!liExt[i])
            return -1;
        if (ext[i] > liExt[i])
            return 1;
        if (ext[i] < liExt[i])
            return -1;
    }

    for (i = 0; i < RODATA_FILENAME_LEN; ++i) {
        if (!name[i])
            return liName[i] != 0;
        if (!liName[i])
            return -1;
        if (name[i] > liName[i])
            return 1;
        if (name[i] < liName[i])
            return -1;
    }
    return 0;
}

rodata_descriptor rodata_find_file(char *name, char *ext) {
    if (!prodata_fs)
        if (!rodata_init())
            return RODATA_INVALID_FILE_DESCRIPTOR;


    /* Номер первого элемента в массиве */
    uint32_t first_file = 0;
    /* Номер элемента в массиве, СЛЕДУЮЩЕГО ЗА последним */
    uint32_t last_file =  FILES_IN_TABLE() - 1;
    rodata_assert(last_file != ~0);

    if (ext[0] < FILE_HADER(first_file).fileExt[0] ||
        ext[0] > FILE_HADER(last_file).fileExt[0])
        return RODATA_INVALID_FILE_DESCRIPTOR;

    /* Если просматриваемый участок непустой, first < last */
    while (first_file < last_file) {
        uint32_t mid = first_file + ((last_file - first_file) >> 2);

        if (is_file_mach(name, ext, mid) <= 0)
            last_file = mid;
        else
            first_file = mid + 1;
    }

    int8_t r = is_file_mach(name, ext, last_file);
    return r ? RODATA_INVALID_FILE_DESCRIPTOR : &prodata_fs->records[last_file];
}

#ifdef CACHING_SUPPORT
uint8_t rodata_readchar(uint32_t offset) {
    uint8_t r;
    read_from_cache(&r, offset, sizeof(uint8_t));
    return r;
}
#endif

uint32_t rodata_readarray(rodata_descriptor descriptor, uint8_t *buf,
                          uint32_t start, uint32_t size) {
    rodata_assert(prodata_fs && descriptor != RODATA_INVALID_FILE_DESCRIPTOR);

    if (start + size > descriptor->size) {
        int32_t _size = descriptor->size - start;
        if (_size <= 0)
            return 0;
        size = _size;
    }

    start += descriptor->startOffset;
    read_boot_flash(start, buf, size);
    return size;
}

uint32_t rodata_readarray_by_pointer(uint8_t *buf, uint32_t start, uint32_t size) {
    read_boot_flash(start, buf, size);
    return size;
}

uint32_t rodata_filesize(rodata_descriptor descriptor) {
    rodata_assert(prodata_fs && descriptor != RODATA_INVALID_FILE_DESCRIPTOR);

    return descriptor->size;
}

uint32_t rodata_filedata_pointerAbsolute(rodata_descriptor descriptor) {
    rodata_assert(prodata_fs && descriptor != RODATA_INVALID_FILE_DESCRIPTOR);

    return descriptor->startOffset + RODATA_BLOB_START;
}
