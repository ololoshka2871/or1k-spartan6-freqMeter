#!/usr/bin/env python
# coding: utf-8

import sys
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mindev","-m",type=int, help="Минимальный делитель (pow2)", required=True)
    parser.add_argument("--maxdev","-M",type=int, help="Максимальный делитель (pow2)", required=True)
    parser.add_argument("--freq","-F",  type=int, help="Входная частота", required=True)
    parser.add_argument("--target","-T",type=int, help="Цель (мс)", required=True)

    args = parser.parse_args()

    design_period_s = args.target / 1000

    for i in range(args.mindev, args.maxdev):
        devider_val = 1 << i
        period_s = devider_val* 1.0 / args.freq
        if (period_s >= design_period_s):
            print(i)
            return

    print(args.maxdev)

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
