#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import socket
import time
import random
import threading


port = 9128
exitapp = False


def print_help():
    print("Возможно вы пытаитесь запустить этот файл напрямую?\n"
    "Запстите: $ TEST_IP=<ip_аддресс> make pb_tesetr.run\n")


def listener(x, sock):
    while not exitapp:
        try:
            conn, addr = sock.recvfrom(1024)
        except:
            return
        print (conn)


def main():
    try:
        import protocol_pb2
    except:
        print("Не удалось ипортировать определения протокола!\n")
        print_help()
        return

    if not ('TEST_IP' in os.environ.keys()):
        print("Не указан IP адрес для теста!\n")
        print_help()
        return

    print('Testing device on IP={}'.format(os.environ['TEST_IP']))

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('', 0))

    ressiver = threading.Thread(target=listener, args=(0, udp_socket))
    ressiver.start()

    ip_addr = os.environ['TEST_IP']

    for test in tests:
        request = test(protocol_pb2)
        udp_socket.sendto(request.SerializeToString(), (ip_addr, port))

    global exitapp
    exitapp = True


# tests
def simplest_test(proto):
    r = proto.Request()
    r.id = random.randrange(0xffffffff)
    r.version = 1
    return r;


tests = (simplest_test, )

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
