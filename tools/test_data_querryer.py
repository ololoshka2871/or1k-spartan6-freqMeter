#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time
import threading
import sys
import struct

SERVER = '192.168.1.99'
PORT = 4998

ADDR=(SERVER, PORT)

exitapp = False

def listener(x, sock):
    while not exitapp:
        try:
            conn, addr = sock.recvfrom(1024)
        except:
            return

        pos = 0
        while not exitapp:
            try:
                v = struct.unpack_from('>Id', conn[pos:pos+12])
                sys.stdout.write("{};{:.2f};".format(v[0], v[1]))
                pos = pos + 12
            except Exception as e:
                sys.stdout.write('\n')
                break

def main():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('', 0))

    ressiver = threading.Thread(target=listener, args=(0, udp_socket))
    ressiver.start()

    while True:
        try:
            time.sleep(0.01)
        except :
            exitapp = True
            print ('Exiting...')
            udp_socket.close()
            return;

        udp_socket.sendto(b'req', ADDR)

# чтобы при импорте не выполнялся код автоматом
if __name__ == '__main__':
    main()
