# -*- coding: utf-8 -*-

import protocol_pb2
import socket
import select
import threading
import random
import time


class TimeoutError(RuntimeError):
    pass


class r4_24_2_requestBuilder:
    @staticmethod
    def build_request():
        """
        Создаёт заготовку запроса

        :return: объект типа protocol_pb2.Request c заполнениыми полями id и version
        """
        req = protocol_pb2.Request()
        req.id = random.randrange(0xffffffff)
        req.protocolVersion = protocol_pb2.INFO.Value('PROTOCOL_VERSION')
        req.deviceID = protocol_pb2.INFO.Value('ID_DISCOVER')
        return req

    @staticmethod
    def build_ping_request():
        """
        Создаёт запрос проверки соединения

        :return: объект типа protocol_pb2.Request
        """
        return r4_24_2_requestBuilder.build_request()

    @staticmethod
    def build_settings_request():
        """
        Создаёт запрос чтения настроек

        :return: объект типа protocol_pb2.Request
        """
        req = r4_24_2_requestBuilder.build_request()
        req.writeSettingsReq.CopyFrom(protocol_pb2.WriteSettingsReq())
        return req

    @staticmethod
    def build_set_time_request():
        """
        Создаёт запрос чтения настроек

        :return: объект типа protocol_pb2.Request
        """
        req = r4_24_2_requestBuilder.build_request()
        ts = protocol_pb2.Timestamp()
        req.setClock.CopyFrom(ts)
        return req

    @staticmethod
    def build_reboot_request(reset_defaults):
        """
        Создаёт запрос на перезагрузку устройства

        :param reset_defaults: Сбросить все настройки на заводсткие?
        :return: объект типа protocol_pb2.Request
        """
        req = r4_24_2_requestBuilder.build_request()
        rr = protocol_pb2.RebootRequest()
        rr.resetDefaults = bool(reset_defaults)
        req.rebootRequest.CopyFrom(rr)
        return req

    @staticmethod
    def build_measure_time_request(chanels_read=None, chanels_write=None):
        """
        Создаёт запрос чтения/записи времени измерения

        :param chanels_read: Список каналов, врмя измерения ктороых необходимо прочитать
        :param chanels_write: dict {номер_канала,
                {measureTime_ms: <value>, chanelEnabled: <False/True>}} кторых необходимо записать
        :return: объект типа protocol_pb2.Request
        """
        req = r4_24_2_requestBuilder.build_request()

        if chanels_read is None:
            read_set = set()
        else:
            read_set = set(chanels_read)

        if chanels_write is None:
            write_set = set()
        else:
            write_set = chanels_write.keys()

        product = protocol_pb2.SetMeasureTimeRequest()

        for r in write_set:
            wr = product.chanelSetMeasureTime.add()
            wr.chanelNumber = r
            d = chanels_write[r]
            if 'measureTime_ms' in d.keys():
                wr.measureTime_ms = d['measureTime_ms']
            if 'chanelEnabled' in d.keys():
                wr.chanelEnabled = d['chanelEnabled']
            if r in read_set:
                read_set.remove(r)

        for r in read_set:
            rr = product.chanelSetMeasureTime.add()
            rr.chanelNumber = r

        req.setMeasureTimeRequest.CopyFrom(product)
        return req

    @staticmethod
    def build_getmeasureresults_request(chanels):
        """
        Создаёт запрос чтения значений измеренной частоты

        :param chanels: dict {номер канала: isVerbose} опрашиваемые каналы
        :return: объект типа protocol_pb2.Request
        """
        req = r4_24_2_requestBuilder.build_request()

        product = protocol_pb2.GetMeasureResultsReq()
        for ch in chanels:
            chanel = product.chanels.add()
            chanel.chanelNumber = ch

        req.getMeasureResultsReq.CopyFrom(product)
        return req


class r4_24_2_io:
    """Класс для простого доступа к РЧ-24 v2 по ротоколу UDP с использованием google protocol buffers"""
    def __init__(self, address, port=9128):
        """
        Конструктор

        :param address: Адрес к которому будет произведено подключение, например '192.168.0.1', device.example.com
        :param port: Порт UDP к которому будет произведено подключение
        """
        self.address = address
        self.port = port
        self.base_timeout = 0.001  # сек
        self.isConnected = False
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setblocking(0)

    def __str__(self):
        """
        Выводит краткую информацию о состоянии драйвера

        :return: Строка с краткой информацией ос состоянии устройства и соединения
        """
        return 'РЧ-24 v2 по адрсесу "{}"'.format(self.address)

    def connect(self):
        """
        Инициирует подключение к устройству

        :return: None
        """
        if self.isConnected:
            raise RuntimeError('Already connected')

        self.udp_socket.bind(('', 0))

        self.isConnected = True

    def disconnect(self):
        """
        Инициирует отключение от устройства

        :return: None
        """
        if not self.isConnected:
            return

        self.udp_socket.close()

    def process_request_sync(self, request, timeout_sec=1):
        """
        Синхронный обработчик запроса (блокирет вызвавший поток до получения ответа или до истечения таймаута)

        :param request: объект типа protocol_pb2.Request
        :param timeout_sec: Таймаут ожидания ответа от устройства
        :return:
        """
        if not (type(request) is protocol_pb2.Request):
            raise TypeError('"request" mast be instance of "protocol_pb2.Request"')

        return self.process_request_common(request, timeout_sec)

    def process_request_common(self, request, timeout_sec):
        self.udp_socket.sendto(request.SerializeToString(), (self.address, self.port))

        response = protocol_pb2.Response()
        while timeout_sec > 0:
            ready = select.select([self.udp_socket], [], [], self.base_timeout)
            if ready[0]:
                conn, adr = self.udp_socket.recvfrom(4096)
                try:
                    response.ParseFromString(conn)
                except Exception:
                    continue

                if (response.id == request.id) and (adr[0] == self.address)\
                        and response.protocolVersion <= protocol_pb2.INFO.Value('PROTOCOL_VERSION')\
                        and response.deviceID == protocol_pb2.INFO.Value('R4_24_2_ID'):
                    return response  # ok
            timeout_sec -= self.base_timeout

        if timeout_sec <= 0:
            raise TimeoutError('Timeout')

    def async_listener(self, request, callback, timeout_sec):
        try:
            result = self.process_request_common(request, timeout_sec)
        except TimeoutError:
            callback(None)
            return
        callback(result)

    def process_request_async(self, request, callback=None, timeout_sec=1):
        """
        Асинхронный обработчик запроса (вызывает callback, по заверешнию)

        :param request:  объект типа protocol_pb2.Request
        :param callback: функция типа foo(response), которая будет вызвана после получения ответа или истечения таймаута
        :param timeout_sec: Таймаут ожидания ответа от устройства
        :return:
        """
        if not (type(request) is protocol_pb2.Request):
            raise TypeError('"request" mast be instance of "protocol_pb2.Request"')
        self.udp_socket.sendto(request.SerializeToString(), (self.address, self.port))

        thread = threading.Thread(target=self.async_listener, args=(self, request, callback, timeout_sec))
        thread.start()

    def ch_ctl(self, channels, propertys):
        ch2enable = {}
        for ch in channels:
            ch2enable[ch] = propertys

        request = r4_24_2_requestBuilder.build_measure_time_request(chanels_write=ch2enable)
        response = self.process_request_sync(request)
        if response.Global_status != protocol_pb2.STATUS.Value('OK'):
            raise RuntimeError('Error {} then trying to set clock on device'.format(response.Global_status))

    def enable_channels(self, chanels):
        self.ch_ctl(chanels, {'chanelEnabled': True})

    def disable_channels(self, chanels):
        self.ch_ctl(chanels, {'chanelEnabled': False})

    def setClock(self, t):
        request = r4_24_2_requestBuilder.build_set_time_request()
        request.setClock.sec = long(t)
        request.setClock.nsec = long((t - request.setClock.sec) * 1000000000)
        response = self.process_request_sync(request)

        if response.Global_status != protocol_pb2.STATUS.Value('OK'):
            raise RuntimeError('Error {} then trying to set clock on device'.format(response.Global_status))

        return response.timestamp.sec + response.timestamp.nsec / 1000000000.0

    def setMeasureTime(self, chanels, measure_time_ms):
        self.ch_ctl(chanels, {'measureTime_ms': measure_time_ms})