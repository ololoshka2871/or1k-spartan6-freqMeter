#!/usr/bin/env python
# coding: utf-8

import sys, struct
import locale

n = [ord('0') + i for i in range(22)]


def main():
    locale.setlocale(locale.LC_ALL, "")
    if len(sys.argv) != 2:
        print('Usage: {} binary_dump'.format(sys.argv[0]))
        exit(1)
    datafile = open(sys.argv[1], 'rb')
    while True:
        try:
            data = datafile.read(1 + 1 + 1 + 8 + 1 + 4 + 1)
            v = struct.unpack_from('>BBBdBIB', data)
        except:
            exit(0)
        if v[0] == ord('#') and v[2] == ord('=') and v[4] == ord('=') and v[6] == ord('$'):
            print('{};{};{}'.format(v[1] - ord('0'), locale.format("%f", v[3]), v[5]))

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
