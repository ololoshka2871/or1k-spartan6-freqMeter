#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import unittest
import libr4_24_2
import protocol_pb2
import struct
import socket


def print_help():
    print("Возможно вы пытаитесь запустить этот файл напрямую?\n"
    "Запстите: $ TEST_IP=<ip_аддресс> make pb_unittest.run\n")


class libr4_24_2_TestCase(unittest.TestCase):
    def setUp(self):
        if not ('TEST_IP' in os.environ.keys()):
            print("Не указан IP адрес для теста!\n")
            print_help()
            self.skipTest('Prepare failed')

        self.device = libr4_24_2.r4_24_2_io(os.environ['TEST_IP'])
        self.device.connect()

    def tearDown(self):
        self.device.disconnect()

    def ip2int(self, addr):
        return struct.unpack("!I", socket.inet_aton(addr))[0]

    def int2ip(self, addr):
        return socket.inet_ntoa(struct.pack("!I", addr))

    def read_settings(self):
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        resp = self.device.process_request_sync(req)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertIsNotNone(resp)
        self.assertEqual(resp.settings.status, protocol_pb2.SettingsResponse.OK)
        return resp.settings

    def restore_settings(self, saved_settings):
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setIPAddr = saved_settings.IPAddr
        req.writeSettingsReq.setIPmask = saved_settings.IPmask
        req.writeSettingsReq.setIPDefaultGateway = saved_settings.IPDefaultGateway
        req.writeSettingsReq.setMAC_Addr = saved_settings.MAC_Addr
        req.writeSettingsReq.setUseDHCP = saved_settings.UseDHCP
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)

    def test_ping(self):
        req = libr4_24_2.r4_24_2_requestBuilder.build_ping_request()
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))

    # ############## settings simple tests ###############

    def test_read_settings(self):
        self.read_settings()

    def test_write_settings_back(self):
        saved_settings = self.read_settings()
        self.restore_settings(saved_settings)

    def test_change_IPsettings(self):
        # save
        saved_settings = self.read_settings()

        # test
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setIPAddr = self.ip2int('192.168.0.198')
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)
        self.assertEqual(resp.settings.IPAddr, req.writeSettingsReq.setIPAddr)

        # restore
        self.restore_settings(saved_settings)
        self.assertEqual(self.read_settings().IPAddr, saved_settings.IPAddr)

    def test_change_IPMasksettings(self):
        # save
        saved_settings = self.read_settings()

        # test
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setIPmask = self.ip2int('255.255.255.128')
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)
        self.assertEqual(resp.settings.IPmask, req.writeSettingsReq.setIPmask)

        # restore
        self.restore_settings(saved_settings)
        self.assertEqual(self.read_settings().IPmask, saved_settings.IPmask)

    def test_change_IPGatewaysettings(self):
        # save
        saved_settings = self.read_settings()

        # test
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setIPDefaultGateway = self.ip2int('192.168.0.35')
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)
        self.assertEqual(resp.settings.IPDefaultGateway, req.writeSettingsReq.setIPDefaultGateway)

        # restore
        self.restore_settings(saved_settings)
        self.assertEqual(self.read_settings().IPDefaultGateway, saved_settings.IPDefaultGateway)

    def test_change_MACAdress(self):
        # save
        saved_settings = self.read_settings()

        # test
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setMAC_Addr = 0x0013261859ca
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)
        self.assertEqual(resp.settings.MAC_Addr, req.writeSettingsReq.setMAC_Addr)

        # restore
        self.restore_settings(saved_settings)
        self.assertEqual(self.read_settings().MAC_Addr, saved_settings.MAC_Addr)

    def test_change_DHCP(self):
        # save
        saved_settings = self.read_settings()

        # test
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setUseDHCP = not saved_settings.UseDHCP
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)
        self.assertEqual(resp.settings.UseDHCP, req.writeSettingsReq.setUseDHCP)

        # restore
        self.restore_settings(saved_settings)
        self.assertEqual(self.read_settings().UseDHCP, saved_settings.UseDHCP)

    # ############## settings complex ###############

    def test_change_setIP_and_MAC(self):
        # save
        saved_settings = self.read_settings()

        # test
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setIPAddr = self.ip2int('192.168.0.198')
        req.writeSettingsReq.setMAC_Addr = 0x0013261859ca
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['OK'].number)
        self.assertEqual(resp.settings.IPAddr, req.writeSettingsReq.setIPAddr)
        self.assertEqual(resp.settings.MAC_Addr, req.writeSettingsReq.setMAC_Addr)

        # restore
        self.restore_settings(saved_settings)
        restored_settings = self.read_settings()
        self.assertEqual(restored_settings.IPAddr, saved_settings.IPAddr)
        self.assertEqual(restored_settings.MAC_Addr, saved_settings.MAC_Addr)

    # ############## settings incorrect ###############

    def test_incorrectMAC(self):
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setMAC_Addr = 0x0a13261859ca
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['ERR_MAC'].number)
        self.assertNotEqual(resp.settings.MAC_Addr, req.writeSettingsReq.setMAC_Addr)

    def test_incorrect_subnetmask(self):
        req = libr4_24_2.r4_24_2_requestBuilder.build_settings_request()
        req.writeSettingsReq.setIPmask = self.ip2int('255.34.255.0')
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('ERRORS_IN_SUBCOMMANDS'))
        self.assertEqual(resp.settings.status,
                         protocol_pb2._SETTINGSRESPONSE_ERRORDESCRIPTION.values_by_name['ERR_NETMASK'].number)
        self.assertNotEqual(resp.settings.IPmask, req.writeSettingsReq.setIPmask)

if __name__ == '__main__':
    unittest.main()
