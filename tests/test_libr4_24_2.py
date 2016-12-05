#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import unittest
import libr4_24_2
import protocol_pb2


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

    def test_ping(self):
        req = libr4_24_2.r4_24_2_requestBuilder.build_ping_request()
        resp = self.device.process_request_sync(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.Global_status, protocol_pb2.STATUS.Value('OK'))

if __name__ == '__main__':
    unittest.main()
