#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import socket
import random
import threading
import select

port = 9128
exitapp = False


def print_help():
    print("Возможно вы пытаитесь запустить этот файл напрямую?\n"
    "Запстите: $ TEST_IP=<ip_аддресс> make pb_tesetr.run\n")


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

    ressiver = threading.Thread(target=listener, args=(0, udp_socket, protocol_pb2))
    ressiver.start()

    ip_addr = os.environ['TEST_IP']

    for test in tests:
        name, request = test(protocol_pb2)
        print('Running {} test'.format(name))
        udp_socket.sendto(request, (ip_addr, port))

    threading._sleep(0.1)

    global exitapp
    exitapp = True


def listener(x, sock, proto):
    response = proto.Response()
    sock.setblocking(0)
    while not exitapp:
        ready = select.select([sock], [], [], 0.3)
        if ready[0]:
            try:
                conn, addr = sock.recvfrom(1024)
            except:
                return

            try:
                response.ParseFromString(conn)
                print(response)
            except:
                print('Protocol error')


# tests
def raise_error(proto):
    return ('Protocol error', "Send string")


def simplest_test(proto):
    r = proto.Request()
    r.id = random.randrange(0xffffffff)
    r.version = 1
    return ('Simple', r.SerializeToString())


tests = (raise_error, simplest_test)

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
