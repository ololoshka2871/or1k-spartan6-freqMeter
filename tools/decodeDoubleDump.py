#!/usr/bin/env python
# coding: utf-8

import sys, struct
import locale


def main():
    locale.setlocale(locale.LC_ALL, "")
    if len(sys.argv) != 2:
        print('Usage: {} binary_dump'.format(sys.argv[0]))
        exit(1)
    datafile = open(sys.argv[1], 'rb')
    while True:
        try:
            data = datafile.read(1 + 1 + 4 + 4 + 4 + 1)
            v = struct.unpack_from('>BBIIIB', data)
        except:
            exit(0)
        if v[0] == ord('#') and v[5] == ord('$'):
            print('{};{};{};{}'.format(v[1], v[2], v[3], v[4]))

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
