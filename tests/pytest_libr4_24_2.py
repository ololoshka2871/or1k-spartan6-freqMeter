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
    "Запстите: $ TEST_IP=<ip_аддресс> make pb_unittest.run\n")


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


def test_prepare_request(device, settings_req):
    assert device
    assert settings_req


def test_ping(device):
    req = libr4_24_2.r4_24_2_requestBuilder.build_ping_request()
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')


# ############## settings simple tests ###############


def test_read_settings(device, settings_req):
    assert read_settings(device, settings_req)


def test_write_settings_back(device, settings_req):
    saved_settings = read_settings(device, settings_req)
    restore_settings(device, saved_settings)


def test_change_IPsettings(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setIPAddr = ip2int('192.168.0.198')
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.IPAddr == req.writeSettingsReq.setIPAddr

    # restore
    restore_settings(device, saved_settings)
    assert read_settings(device, settings_req).IPAddr == saved_settings.IPAddr

def test_change_IPMasksettings(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setIPmask = ip2int('255.255.255.128')
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.IPmask == req.writeSettingsReq.setIPmask

    # restore
    restore_settings(device, saved_settings)
    assert read_settings(device, settings_req).IPmask == saved_settings.IPmask


def test_change_IPGatewaysettings(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setIPDefaultGateway = ip2int('192.168.0.35')
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.IPDefaultGateway == req.writeSettingsReq.setIPDefaultGateway

    # restore
    restore_settings(device, saved_settings)
    assert read_settings(device, settings_req).IPDefaultGateway == saved_settings.IPDefaultGateway


def test_change_MACAdress(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setMAC_Addr = 0x0013261859ca
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.MAC_Addr == req.writeSettingsReq.setMAC_Addr

    # restore
    restore_settings(device, saved_settings)
    assert read_settings(device, settings_req).MAC_Addr == saved_settings.MAC_Addr

def test_change_DHCP(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setUseDHCP = not saved_settings.UseDHCP
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.UseDHCP == req.writeSettingsReq.setUseDHCP

    # restore
    restore_settings(device, saved_settings)
    assert read_settings(device, settings_req).UseDHCP == saved_settings.UseDHCP

# ############## settings complex ###############


def test_change_setIP_and_MAC(device, settings_req):
    # save
    saved_settings = read_settings(device, settings_req)

    # test
    req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
    req.writeSettingsReq.setIPAddr = ip2int('192.168.0.198')
    req.writeSettingsReq.setMAC_Addr = 0x0013261859ca
    resp = device.process_request_sync(req)
    assert resp
    assert resp.Global_status == protocol_pb2.STATUS.Value('OK')
    assert resp.settings.status == protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number
    assert resp.settings.IPAddr == req.writeSettingsReq.setIPAddr
    assert resp.settings.MAC_Addr == req.writeSettingsReq.setMAC_Addr

    # restore
    restore_settings(device, saved_settings)
    restored_settings = read_settings(device, settings_req)
    assert restored_settings.IPAddr == saved_settings.IPAddr
    assert restored_settings.MAC_Addr == saved_settings.MAC_Addr

# ############## settings incorrect ###############

@pytest.mark.parametrize("test_mac,result",
    [(0x0a13261859ca, False),
     (0x0a0000000000, False),
     (0x013dac5e3270, False),
     (0x0013261859ca, True)])
def test_incorrectMAC(device, settings_req, test_mac, result):
    settings_req.writeSettingsReq.setMAC_Addr = test_mac
    resp = device.process_request_sync(settings_req)
    assert resp
    assert (resp.Global_status != protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS')) == result
    assert (resp.settings.status !=
            protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['ERR_MAC'].number) == result
    assert (resp.settings.MAC_Addr == settings_req.writeSettingsReq.setMAC_Addr) == result


@pytest.mark.parametrize("test_mask,result",
    [(ip2int('255.34.255.0'), False),
     (ip2int('0.255.255.0'), False),
     (ip2int('255.255.11.0'), False),
     (ip2int('255.255.254.0'), True)])
def test_incorrect_subnetmask(device, settings_req, test_mask, result):
    settings_req.writeSettingsReq.setIPmask = test_mask
    resp = device.process_request_sync(settings_req)
    assert resp
    assert (resp.Global_status != protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS')) == result
    assert (resp.settings.status !=
            protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['ERR_NETMASK'].number) == result
    assert (resp.settings.IPmask == settings_req.writeSettingsReq.setIPmask) == result


@pytest.mark.parametrize("test_mask,test_gateway,result",
    [(ip2int('255.255.255.0'), ip2int('192.168.32.11'), False),
     (ip2int('255.255.0.0'), ip2int('192.163.0.26'), False),
     (ip2int('255.255.255.0'), ip2int('192.168.0.1'), True)])
def test_incorrect_gateway_subnetmask(device, settings_req, test_mask, test_gateway, result):
    settings_req.writeSettingsReq.setIPmask = test_mask
    settings_req.writeSettingsReq.setIPDefaultGateway = test_gateway
    resp = device.process_request_sync(settings_req)
    assert resp
    assert (resp.Global_status != protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS')) == result
    assert (resp.settings.status !=
            protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['ERR_GATEWAY'].number) == result
    assert (resp.settings.IPmask == settings_req.writeSettingsReq.setIPmask) == result
    assert (resp.settings.IPDefaultGateway == settings_req.writeSettingsReq.setIPDefaultGateway) == result


# TODO


# ############ Test reboot ##################################


def test_reboot(device):
    reboot_req = libr4_24_2.r4_24_2_requestBuilder.build_reboot_request(True)
    try:
        device.process_request_sync(reboot_req)
    except libr4_24_2.TimeoutError:
        pass

