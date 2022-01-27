import time
import serial

#from GripperListenerI import *
from .GripperListenerI import GripperListenerI
from typing import List
from threading import Thread
import csv
import os
import sys
import glob


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
    __GET_COMPLETING_MOVE = 45
    __GET_GRIPPER_ID = 46

    __BACK_POSITION = 50
    __BACK_LOAD = 60
    __BACK_VOLTAGE = 70
    __BACK_TEMPERATURE = 80
    __LAST_MOVE_STATUS = 85
    __BACK_GRIPPER_ID = 86

    __MAX_SPEED = 1023
    __MIN_SPEED = 0

    __MAX_ANGLE = 1023
    __MIN_ANGLE = 0

    __listeners: List[GripperListenerI] = []

    def __init__(self, gripper_id: int, baud_rate: int):
        serial_port_list = self.__serial_ports()
        self.ser = None
        self.__listening_th = None
        for port in serial_port_list:
            self.gripper_id = -1
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=1)
                self.start_listening()
                self.get_id()
                time.sleep(0.1)
                self.__listening_th = None
                time.sleep(0.01)
                if gripper_id == self.gripper_id:
                    print("Gripper with id " + str(self.gripper_id) + " found")
                    break
                else:
                    self.__del__()
            except:
                None

        if self.ser == None:
            raise Exception("Gripper with id " + str(gripper_id) + " not found")
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        self.last_move_status = False

    def __del__(self):
        self.__listening_th = None
        if self.ser is not None:
            self.ser.close()
        self.ser = None

    def __serial_ports(self):
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        else:
            raise EnvironmentError('Unsupported platform')
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except(OSError, serial.SerialException):
                pass
        return result

    def __listening(self) -> None:
        """
        Функция приема входящих пакетов и запуска обработчиков
        """
        while self.__listening_th is not None:
            in_wait = self.ser.in_waiting
            while in_wait < 8:
                time.sleep(0.01)
                in_wait = self.ser.in_waiting
                if self.__listening_th is None:
                    return
            else:
                incoming_bytes = self.ser.read(2)
                if incoming_bytes[0] != GripperSerialController.__START_PACKAGE_FLAG or incoming_bytes[
                    1] != GripperSerialController.__START_PACKAGE_FLAG:
                    break
                incoming_bytes += self.ser.read(6)
                val_left = (incoming_bytes[3] << 8) + incoming_bytes[4]
                val_right = (incoming_bytes[5] << 8) + incoming_bytes[6]
                code = incoming_bytes[2]
                if code == self.__BACK_GRIPPER_ID:
                    self.gripper_id = val_left
                elif code == self.__LAST_MOVE_STATUS:
                    if val_left == 1 and val_right == 1:
                        self.last_move_status = True
                    else:
                        self.last_move_status = False
                elif code == self.__BACK_TEMPERATURE:
                    val_left = val_left
                    val_right = val_right
                elif code == self.__BACK_POSITION:
                    val_left = val_left / 1023 * 300
                    val_right = val_right / 1023 * 300
                elif code == self.__BACK_VOLTAGE:
                    val_left = val_left / 10.0
                    val_right = val_right / 10.0
                elif code == self.__BACK_LOAD:
                    val_left = val_left * 100 / 1023
                    val_right = val_right * 100 / 1023
                for listener in self.__listeners:
                    listener.process_data(incoming_bytes, code, val_left, val_right)

    def start_listening(self):
        """
        Функция запуска обработчиков сообщений
        """
        self.__listening_th = Thread(target=self.__listening, args=(), daemon=True)
        self.__listening_th.start()

    def open(self):
        """
        Функция открытия гриппера со стандартной скоростью в стандартное открытое положение
        """
        self.__send_message(self.__make_message(type_package=self.__SEND_OPEN))

    def close(self):
        """
        Функция закрытия гриппера со стандартной скоростью.
        Сдвигает губки в базовое закрытое положение,
        усилие сжатия не регулируется и достаточно мало
        """
        self.__send_message(self.__make_message(type_package=self.__SEND_CLOSE))

    def release(self):
        """
        Разблокировать двигатели
        (Перевести двигатели в режим вращения с нулевыми скоростью/уcилием)
        """
        self.__send_message(self.__make_message(type_package=self.__SEND_RELEASE))

    def unrelease(self):
        """
        Заблокирвоать двигатели
        Перевести двигатели в режим удержания угла (при вызове фиксируются в текущем положении)
        """
        self.__send_message(self.__make_message(type_package=self.__SEND_UNRELEASE))

    def open_speed(self, speed: int):
        """
        Функция открытия гриппера со заданной скоростью
        """
        speed = int(speed * 1023 / 100)
        speed = ((speed <= 1023) and (speed >= 0) * speed) + (1023 * speed > 1023)
        self.__send_message(self.__make_message(type_package=self.__SEND_OPEN_TORQUE, val1=speed))

    def close_torque(self, torque: int):
        """
        Функция закрытия гриппера со заданынм усилием
        """
        torque = int(torque * 1023 / 100)
        torque = ((torque <= 1023) and (torque >= 0) * torque) + (1023 * (torque > 1023))
        self.__send_message(self.__make_message(type_package=self.__SEND_CLOSE_TORQUE, val1=torque))

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

    def get_id(self):
        """
        Функция получения ID гриппера (не динамикселей) .
        Значение возвращается с кодом __GET_GRIPPER_ID
        """
        self.__send_message(self.__make_message(type_package=self.__GET_GRIPPER_ID))

    def get_completing_lact_command(self):
        """
        Функция получения статуса последней команды.
        Значение возвращается внещнему обработчику (`__listeners`) с кодом __LAST_MOVE_STATUS и в переменную last_move_status
        """
        self.__send_message(self.__make_message(type_package=self.__GET_COMPLETING_MOVE))

    def __send_message(self, message: bytes):
        if 100 <= message[2] <= 150:
            self.last_move_status = False
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
        while os.path.exists('./' + str(postfix_counter) + "_" + filename):
            postfix_counter += 1
        self.__filename = str(postfix_counter) + "_" + filename
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
        # Создание экземпляра гриппера на заданном порту
        gripper = None
        for i in range(10):
            try:
                gripper = GripperSerialController(i, 57600)
            except:
                continue
        # Подключние и запуск обработчиков входящих сообщений
        gripper.attach(listener=Printer())
        gripper.attach(listener=CSVPrinter('output.csv'))
        gripper.start_listening()
        time.sleep(3)
        gripper.release()
        gripper.get_completing_lact_command()
        time.sleep(0.2)
        print(gripper.last_move_status)
        time.sleep(3)
        gripper.get_completing_lact_command()
        # Простое закрытие-открытие гриппера
        gripper.close()
        time.sleep(3)
        print(gripper.last_move_status)
        gripper.open()
        time.sleep(3)
        print(gripper.last_move_status)
        # Закрытие с заданным усилием и открытие с заданной скоростью из диапазона 0 - 100
        gripper.close_torque(30)
        time.sleep(3)
        gripper.open_speed(30)
        time.sleep(3)
        # Ослабить для свободного перещения и зафиксировать в текущем положении губки гриппера
        gripper.release()
        time.sleep(3)
        gripper.unrelease()
        time.sleep(3)

        # Запросить нагрузку, позицию, напряжение и температуру с двигателя (обрабатывается слушателями)
        # А так же статус выполнения последней команды на передвижение (записывается в переменную last_move_status)
        gripper.get_load()
        gripper.get_position()
        gripper.get_voltage()
        gripper.get_temp()
        time.sleep(3)

        gripper.open()
        time.sleep(4)
        gripper.release()
        # gripper2.release()
    except KeyboardInterrupt:
        gripper = GripperSerialController(5, 57600)
        gripper.release()
        print("Stopped by KeyboardInterrupt")
