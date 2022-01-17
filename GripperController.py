import time

import serial
from .GripperListenerI import GripperListenerI
from typing import List
from threading import Thread
import csv
import os


class GripperSerialController(object):
    __START_PACKAGE_FLAG = 255

    __SEND_RELEASE = 100
    __SEND_UNRELEASE = 101
    __SEND_OPEN = 110
    __SEND_CLOSE = 111
    __SEND_OPEN_TORQUE = 120
    __SEND_CLOSE_TORQUE = 121
    __SEND_ANGLE = 130
    __SEND_LEFT_ANGLE = 131
    __SEND_RIGHT_ANGLE = 132
    __SEND_ANGLE_SPEED = 140
    __SEND_LEFT_ANGLE_SPEED = 141
    __SEND_RIGHT_ANGLE_SPEED = 142

    __GET_POSITION = 10
    __GET_LOAD = 20
    __GET_VOLTAGE = 30
    __GET_TEMPERATURE = 40
    __BACK_POSITION = 50
    __BACK_LOAD = 60
    __BACK_VOLTAGE = 70
    __BACK_TEMPERATURE = 80

    __MAX_SPEED = 1023
    __MIN_SPEED = 0

    __MAX_ANGLE = 1023
    __MIN_ANGLE = 0

    __instance = None

    __listeners: List[GripperListenerI] = []

    def __new__(cls, serial_port: str, baud_rate: int):
        if cls.__instance is None:
            cls.__instance = super(GripperSerialController, cls).__new__(cls)
            cls.__instance.__initialized = False
        return cls.__instance

    def __init__(self, serial_port: str, baud_rate: int):
        if self.__initialized:
            return
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()

    def __listening(self) -> None:
        """
        Функция приема входящих пакетов и запуска обработчиков
        """
        while True:
            in_wait = self.ser.in_waiting
            while in_wait < 8:
                time.sleep(0.01)
                in_wait = self.ser.in_waiting
            if len(self.__listeners) == 0:
                return
            else:
                incoming_bytes = self.ser.read(2)
                if incoming_bytes[0] != GripperSerialController.__START_PACKAGE_FLAG or incoming_bytes[
                    1] != GripperSerialController.__START_PACKAGE_FLAG:
                    break
                incoming_bytes += self.ser.read(6)
                val_left = (incoming_bytes[3] << 8) + incoming_bytes[4]
                val_right =  (incoming_bytes[5] << 8) + incoming_bytes[6]
                code = incoming_bytes[2]
                if code == self.__BACK_TEMPERATURE:
                    val_left = val_left
                    val_right = val_right
                elif code == self.__BACK_POSITION:
                    val_left = val_left/1023*300
                    val_right = val_right/1023*300
                elif code == self.__BACK_VOLTAGE:
                    val_left = val_left/10.0
                    val_right = val_right/10.0
                elif code == self.__BACK_LOAD:
                    val_left = val_left*100/1023-100
                    val_right = val_right*100/1023-100
                for listener in self.__listeners:
                    listener.process_data(incoming_bytes, code,val_left,val_right)

    def start_listening(self):
        """
        Функция запуска обработчиков сообщений
        """
        th = Thread(target=self.__listening, args=(),daemon=True)
        th.start()

    def open(self):
        """Функция открытия гриппера со стандартной скоростью"""
        self.__send_message(self.__make_message(type_package=self.__SEND_OPEN))

    def close(self):
        """Функция закрытия гриппера со стандартной скоростью"""
        self.__send_message(self.__make_message(type_package=self.__SEND_CLOSE))

    def release(self):
        self.__send_message(self.__make_message(type_package=self.__SEND_RELEASE))

    def unrelease(self):
        self.__send_message(self.__make_message(type_package=self.__SEND_UNRELEASE))

    def open_torque(self, speed: int):

        self.__send_message(self.__make_message(type_package=self.__SEND_OPEN_TORQUE, val1=speed))

    def close_torque(self, speed: int):
        """Функция закрытия гриппера со заданынм усилием"""
        self.__send_message(self.__make_message(type_package=self.__SEND_CLOSE_TORQUE, val1=speed))

    def angle(self, angle: int):
        self.__send_message(self.__make_message(type_package=self.__SEND_ANGLE, val2=angle))

    def angle_speed(self, angle: int, speed: int):
        self.__send_message(self.__make_message(type_package=self.__SEND_ANGLE_SPEED, val1=speed, val2=angle))

    def left_angle(self, angle: int):
        self.__send_message(self.__make_message(type_package=self.__SEND_LEFT_ANGLE, val2=angle))

    def right_angle(self, angle: int):
        self.__send_message(self.__make_message(type_package=self.__SEND_RIGHT_ANGLE, val2=angle))

    def left_angle_speed(self, angle: int, speed: int):
        self.__send_message(self.__make_message(type_package=self.__SEND_LEFT_ANGLE_SPEED, val1=speed, val2=angle))

    def right_angle_speed(self, angle: int, speed: int):
        self.__send_message(self.__make_message(type_package=self.__SEND_RIGHT_ANGLE_SPEED, val1=speed, val2=angle))

    def __make_message(self, type_package: int, val1: int = 0, val2: int = 0):
        """Функция создания пакета из 8бит для передачи по Serial"""
        val1_16 = val1 >> 8
        val1_8 = val1 & 255
        val2_16 = val2 >> 8
        val2_8 = val2 & 255
        checksum = (((val1_16 + val1_8 + val2_16 + val2_8 + type_package) & 255) | 1) - 1
        package = bytes([255, 255, type_package, val1_16, val1_8, val2_16, val2_8, checksum])
        return package

    def get_load(self):
        """
        Функция получения нагрузки на губках гриппера.
        Значение возвращается внещнему обработчику (`__listeners`) с кодом __BACK_LOAD
        """
        self.__send_message(self.__make_message(type_package=self.__GET_LOAD))

    def get_voltage(self):
        """
        Функция получения напряжения на двигателях.
        Значение возвращается внещнему обработчику (`__listeners`) с кодом __BACK_VOLTAGE
        """
        self.__send_message(self.__make_message(type_package=self.__GET_VOLTAGE))

    def get_position(self):
        """
        Функция получения напряжения на двигателях.
        Значение возвращается внещнему обработчику (`__listeners`) с кодом __BACK_POSITION

        """
        self.__send_message(self.__make_message(type_package=self.__GET_POSITION))

    def get_temp(self):
        """
        Функция получения температуры динамикселей в градусах цельсия.
        Значение возвращается внещнему обработчику (`__listeners`) с кодом __BACK_TEMPERATURE
        """
        self.__send_message(self.__make_message(type_package=self.__GET_TEMPERATURE))

    def __send_message(self, message: bytes):
        self.ser.write(message)
        self.ser.flushOutput()

    def attach(self, listener: GripperListenerI) -> None:
        print("Subject: Attached an observer.")
        self.__listeners.append(listener)


class Printer(GripperListenerI):
    def process_data(self, package: bytes, type_code: int, left_val: int, right_val: int) -> None:
        print(package.hex(":") + " -> " + str(type_code) + ", " + str(left_val) + ", " + str(right_val))


class CSVPrinter(GripperListenerI):
    __fieldnames = ['time', 'package', 'type', 'val1', 'val2']

    def __init__(self, filename: str):
        postfix_counter = 1
        while os.path.exists('./'+str(postfix_counter)+"_"+ filename):
            postfix_counter+=1
        self.__filename = str(postfix_counter)+"_" +  filename
        with open(self.__filename, 'w', newline='') as csvfile:
            self.__writer = csv.DictWriter(csvfile, fieldnames=self.__fieldnames)
            self.__writer.writeheader()

    def process_data(self, package: bytes, type_code: int, left_val: int, right_val: int) -> None:
        with open(self.__filename, 'a', newline='') as csvfile:
            self.__writer = csv.DictWriter(csvfile, fieldnames=self.__fieldnames)
            self.__writer.writerow(
                {'time': time.time(), 'package': package.hex(":"), 'type': type_code, "val1": left_val,
                 "val2": right_val})


if __name__ == '__main__':
    try:
        gripper = GripperSerialController('/dev/ttyACM0', 57600)
        gripper.attach(listener=Printer())
        gripper.attach(listener=CSVPrinter('output.csv'))
        gripper.start_listening()
        gripper.release()
        time.sleep(5)
        print("Closing wiht torque 0.5")
        gripper.close_torque(300)

        for i in range(100):
            gripper.get_load()
            gripper.get_position()
            gripper.get_voltage()
            gripper.get_temp()
            time.sleep(0.1)
        gripper.unrelease()
        time.sleep(2)
        gripper.open()
        time.sleep(2)
        gripper.release()
    except KeyboardInterrupt:
        gripper = GripperSerialController('/dev/ttyACM0', 57600)
        gripper.release()
        print("Stopped by KeyboardInterrupt")
