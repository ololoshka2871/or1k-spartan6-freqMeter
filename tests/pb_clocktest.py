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


@pytest.fixture
def settime_req():
    return libr4_24_2.r4_24_2_requestBuilder.build_set_time_request()


def ip2int( addr):
    return struct.unpack("!I", socket.inet_aton(addr))[0]


def int2ip(addr):
    return socket.inet_ntoa(struct.pack("!I", addr))


@pytest.fixture
def settings_req():
    return libr4_24_2.r4_24_2_requestBuilder.build_settings_request()

def read_settings(device, settings_req):
    resp = device.process_request_sync(settings_req)
    assert resp
    assert resp.settings.status == protocol_pb2.SettingsResponse.OK
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    return resp.settings


# ############## settings simple tests ###############

def test_clock_set(device, settime_req):
    req = libr4_24_2.r4_24_2_requestBuilder.build_ping_request()
    resp = device.process_request_sync(req)
    print('Current clock value: {}'.format(datetime.datetime.fromtimestamp(resp.timestamp/1000.0)))

    settime_req.setClock = long(time.time()  * 1000)
    print('Syncronisating clocks...')
    resp = device.process_request_sync(settime_req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert (resp.timestamp - settime_req.setClock) < 1000
    print('ok...')
