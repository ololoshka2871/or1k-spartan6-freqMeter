#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import struct

FILL_LINE_PATTERN="    .INIT_{:02X} ( 256'h{:08X}{:08X}{:08X}{:08X}{:08X}{:08X}{:08X}{:08X} ),"

def create_one_string(number, data):
    r = struct.unpack('<IIIIIIII', data)
    return FILL_LINE_PATTERN.format(number, *r)

def create_one_image(data):
    if len(data) % 32 != 0:
        raise ValueError('len(data) % 32 != 0')

    res = []
    for i in range(int(len(data) / 32)):
        tmp = bytearray(data[i*32:(i+1)*32])
        tmp.reverse()
        res.append(create_one_string(i, tmp))

    return '\n'.join(res)

def create_one_instance(data, template, number):
    block_init_text = create_one_image(data)

    template = template.replace('@inst_num@', str(number))
    return template.replace('/*INITVALUES*/', block_init_text)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--templatemodule","-M",type=str, help="Шаблон verilog", required=True)
    parser.add_argument("--templateinstance","-I",type=str, help="Шаблон verilog", required=True)
    parser.add_argument("--image","-i",type=str, help="Образ памяти .bin", required=True)
    parser.add_argument("--blocks","-B",  type=int, help="Количество блоков памяти для генерации", required=True)
    parser.add_argument("--blocksize","-s",type=int, help="Размер каждого блока", required=True)

    args = parser.parse_args()

    # read templates
    main_template = open(args.templatemodule).read()
    template_instance = open(args.templateinstance).read()

    # read data
    image = open(args.image, 'rb').read()
    if len(image) < args.blocks * args.blocksize:
        image = image + b''.join([b'\x00' for i in range(args.blocks * args.blocksize - len(image))])
    else:
        image = image[:args.blocks * args.blocksize]

    #generate blocks
    blocks = []
    for i in range(args.blocks):
        start = i * args.blocksize
        stop = (i + 1) * args.blocksize
        blocks.append(create_one_instance(image[start:stop], template_instance, i))

    blocks = '\n//////////////////////////////////////////////////////////////\n\n'.join(blocks)

    print(main_template.replace('/*PLACEHOLDER*/', blocks))

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
