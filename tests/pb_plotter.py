#!/usr/bin/env python
# -*- coding: utf-8 -*-


import argparse
import os
import sys
import time
import libr4_24_2
import protocol_pb2
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Scope(object):
    def __init__(self, ax):
        self.ax = ax
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.autoscale_view(True,True,True)
        self.points_count = 0

    def update(self, v):
        maxpoints = 100
        self.points_count += 1
        if self.points_count > 100:  # reset the arrays
            self.points_count = 100
            self.tdata = self.tdata[1:]
            self.ydata = self.ydata[1:]

        self.tdata.append(v[0])
        self.ydata.append(v[1])
        self.line.set_data(self.tdata, self.ydata)

        self.ax.relim()
        self.ax.autoscale_view(True,True,True)
        return self.line,

def reader():
    global device
    global request
    while True:
        start = time.time()
        response = device.process_request_sync(request)
        req_time = time.time()
        if (not response) or response.Global_status != protocol_pb2.STATUS.Value('OK'):
            raise RuntimeError('Error {} during read values'.format(response.Global_status))

        r = response.getMeasureResultsResponce.results._values[0]
        F = r.Frequency
        if not r.chanelEnabled:
            F = float('nan')
        yield (r.timestamp, F)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--interval', '-i', type=float, help="Интервал опроса [c]", default=0)
    parser.add_argument('--chanel', '-c', type=int, help="Канал (0 - 23)", default=0)
    parser.add_argument('--measure_time', '-m', type=int, help="Время измерения [мс]", default=10)
    parser.add_argument('--port', '-p', type=int, help='UDP port', default=9128)

    args = parser.parse_args()
    if not ('TEST_IP' in os.environ.keys()):
        print("Не указан IP адрес для теста!\n")
        print("Возможно вы пытаитесь запустить этот файл напрямую?\n"
              "Запстите: $ TEST_IP=<ip_аддресс> make pb_reader.run\n")
        return 1

    global request
    request = libr4_24_2.r4_24_2_requestBuilder.build_getmeasureresults_request([args.chanel])
    global device
    device = libr4_24_2.r4_24_2_io(os.environ['TEST_IP'])
    device.connect()

    device.enable_channels([args.chanel])
    device.setMeasureTime([args.chanel], args.measure_time)
    #device.setClock(time.time())

    time.sleep(2 * args.measure_time / 1000.0)

    fig, ax = plt.subplots()
    scope = Scope(ax)

    ani = animation.FuncAnimation(fig, scope.update, reader, interval=args.measure_time,
                                  blit=False)
    plt.show()


# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
