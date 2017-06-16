#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import pytest
import libr4_24_2
import protocol_pb2
import struct
import socket
import time
import datetime


def print_help():
    print("Возможно вы пытаитесь запустить этот файл напрямую?\n"
    "Запстите: $ TEST_IP=<ip_аддресс> make pb_netmasktest.run\n")


@pytest.fixture
def device(request):
    if not ('TEST_IP' in os.environ.keys()):
        print("Не указан IP адрес для теста!\n")
        print_help()
        assert 0

    d = libr4_24_2.r4_24_2_io(os.environ['TEST_IP'])
    d.connect()

    def fin():
        d.disconnect()

    request.addfinalizer(fin)
    return d

# ############## settings simple tests ###############

def test_chanel_enable_disable(device):
    chanel = 0
    #disable chanel
    ch2dis_dict = {chanel: {'chanelEnabled': False}}
    req = libr4_24_2.r4_24_2_requestBuilder.build_measure_time_request(chanels_write=ch2dis_dict)
    resp = device.process_request_sync(req)
    assert resp
    assert (resp.Global_status != protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS'))

    time.sleep(1)

    #endble chanel

    ch2en_dict = {chanel: {'chanelEnabled': True}}
    req = libr4_24_2.r4_24_2_requestBuilder.build_measure_time_request(chanels_write=ch2en_dict)
    resp = device.process_request_sync(req)
    assert resp
    assert (resp.Global_status != protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS'))
