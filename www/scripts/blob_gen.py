#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
import re
import struct
import zlib

def get_directory_content(root):
    result = []
    for d, firs, files in os.walk(root):
        for f in files:
            path = os.path.join(d, f)
            result.append(path)
    return result

def filter_stings(strings, filter):
    result = []
    for s in strings:
        if not filter.search(s):
            result.append(s)
    return result

def read_input_files(files_list, rootdir, path_max, ext_max):
    records = {}
    for f in files_list:
        name = f.replace(rootdir, '')
        if (name[0] == '/'):
            name = name[1:]
        fname, ext = os.path.splitext(name)
        ext = ext[1:]
        if len(fname) > path_max or len(ext) > ext_max:
            raise RuntimeError('{} не удовлетворяет соглашению {}.{}'.format(name, path_max, ext_max))
        rf = open(f, 'rb')
        content = rf.read()
        rf.close()
        records[name] = (ext, fname, content)

    return records

def sort_records(records, ext_len):
    map_ext2filename = []
    for key in records.keys():
        value = records[key]
        name = value[1]
        extantion = value[0]

        sort_ready = extantion
        if len(sort_ready) < ext_len:
            sort_ready = extantion + '\\x0' * (ext_len - len(extantion))
        sort_ready = sort_ready + name

        map_ext2filename.append((sort_ready, key))

    map_ext2filename = sorted(map_ext2filename)

    return list(map(lambda x: x[1], map_ext2filename))

def format_hader(records, order, name_size, ext_size, is_le):
    hhader_len = 16
    reserved_field_len = 16 - 2 * 4

    if is_le:
        endian_def = '<'
    else:
        endian_def = '>'

    struct_def = endian_def + str(ext_size) + 's' + str(name_size) + 's' + 'II'

    hader_size = (ext_size + name_size + 2 * 4) * len(order) + hhader_len

    offset = hader_size

    print("Построение заголовка блоба, структура {}".format(struct_def))
    hader_records = b''
    for record in order:
        name = records[record][1]
        ext = records[record][0]
        start = offset
        size = len(records[record][2])
        print('{{ ext: {}\tname: {}\tstart: {}\tsize:{} }}'.format(ext, name, start, size))
        offset += size

        hader_records += struct.pack(struct_def, ext.encode('utf-8'), name.encode('utf-8'), start, size)

    result  = struct.pack(endian_def + 'II', hader_size, 0) + b'\00' * reserved_field_len
    result += hader_records

    hader_crc32 = zlib.crc32(result)
    print('Контрольная сумма заголовка: 0x{:08X}'.format(hader_crc32))

    result = result[0:4] + struct.pack(endian_def + 'I', hader_crc32) + result[8:]

    return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--outputblob", '-o', type=str, help="Двоичный блоб-файл", required=True)
    parser.add_argument("--path_max", type=int, help="Максимальная длина путь + имя", default=21)
    parser.add_argument("--ext_max", type=int, help="Максимальная длина расширения", default=3)
    parser.add_argument("--ignore_regexp", type=str, help="Игнорировать файлы по regexp", default='')
    parser.add_argument("--little_ed", '-l', action='store_true', help='Little endian' , default=False)
    parser.add_argument("rootdir",type=str, help="Каталог для обработки")

    args = parser.parse_args()

    if (args.ignore_regexp) != '':
        filter = re.compile(args.ignore_regexp)
    else:
        filter = None

    files_list = get_directory_content(args.rootdir)
    if len(files_list) == 0:
        printf('Каталог {} пуст'.format(args.rootdir))
        exit(1)

    if (filter):
        files_list = filter_stings(files_list, filter)
        if len(files_list) == 0:
            print('Фильтрация удалила все возможные записи, нечего обрабатывать')
            exit(2)

    records = read_input_files(files_list, args.rootdir, args.path_max, args.ext_max)
    _order = sort_records(records, args.ext_max)

    resfile = open(args.outputblob, 'wb')
    resfile.write(format_hader(records, _order, args.path_max, args.ext_max, args.little_ed))

    for record in _order:
        resfile.write(records[record][2])

    print('Генерация блоба завершена!')
    resfile.close()

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
