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
        while True:
            if datafile.read(1) == b'$':
                break;
        try:
            data = datafile.read(1 + 1 + 1)
            v = struct.unpack_from('>BBB', data)
        except:
            exit(0)
        if v[0] == ord('#') and v[2] == ord('$'):
            print('{}'.format(v[1]))

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
