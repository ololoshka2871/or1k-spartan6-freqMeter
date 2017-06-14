#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import pytest
import libr4_24_2
import protocol_pb2
import struct
import socket
import time


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


def restore_settings(device, saved_settings):
    req = settings_req()
    req.writeSettingsReq.setIPAddr = saved_settings.IPAddr
    req.writeSettingsReq.setIPmask = saved_settings.IPmask
    req.writeSettingsReq.setIPDefaultGateway = saved_settings.IPDefaultGateway
    req.writeSettingsReq.setMAC_Addr = saved_settings.MAC_Addr
    req.writeSettingsReq.setUseDHCP = saved_settings.UseDHCP
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number

# ############## settings simple tests ###############

def test_change_IPMasksettings(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)
    print('Original Netmask: {}'.format(int2ip(saved_settings.IPmask)))

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setIPmask = ip2int('255.255.255.128')
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.IPmask == req.writeSettingsReq.setIPmask

    print('Writen Netmask: {}'.format(int2ip(resp.settings.IPmask)))

    # restore
    restore_settings(device, saved_settings)
    restored = read_settings(device, settings_req).IPmask
    assert restored == saved_settings.IPmask
    print('Restored Netmask: {}'.format(int2ip(restored)))
