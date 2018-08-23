#!/usr/bin/env python
# encoding: utf-8

B_I2C_ADR = 0x08

B_I2C_GPIO_ANALOG_START_L = 13
B_I2C_GPIO_CONTROL_START = 19
B_I2C_GPIO_START = 23

B_PWM_OFFSET = 256
B_PWM_RANGE = 256

INPUT = 0
OUTPUT = 1
INPUT_ANALOG = 2
INPUT_PULLUP = 3


class BlackJackBoardPWM(object):
    def __init__(self, bus):
        self._bus = bus
        self._pwm_pin_map = {
            1: 3,
            2: 1,
            3: 2,
            4: 4
        }

    def __getitem__(self, key):
        if key < 0 or key > 3:
            raise IndexError("PWM index must be between 0 and 3")
        key = self._pwm_pin_map[key + 1] - 1
        command = (2 * key) + 1

        value = self._bus.read_byte_data(B_I2C_ADR, command) + (self._bus.read_byte_data(B_I2C_ADR, command + 1) << 7)
        return (value - B_PWM_OFFSET) * 100.0 / B_PWM_RANGE

    def __setitem__(self, key, percent):
        if key < 0 or key > 3:
            raise IndexError("PWM index must be between 0 and 3")
        key = self._pwm_pin_map[key + 1] - 1
        command = (2 * key) + 1

        value = int((percent / 100.0) * B_PWM_RANGE) + B_PWM_OFFSET

        # high = (value & 0b1111111000) >> 3
        # low = value & 0b0000000111

        high = value >> 7
        low = value & 0x7F

        # print
        # print value
        # print "H:", bin(high)
        # print "L:", bin(low)
        # print "Setting Servo", key + 1, "to", percent, "% [ PWM:", value, "]"

        self._bus.write_byte_data(B_I2C_ADR, command, low)
        self._bus.write_byte_data(B_I2C_ADR, command + 1, high)


class BlackJackBoardGPIO(object):
    def __init__(self, bus):
        self._bus = bus
        self._pin_map = {
            4: 1,
            3: 2,
            2: 4,
            1: 3,
        }

    def pin_mode(self, pin, mode):
        pin = self._pin_map[pin]
        if pin == 2 and mode == INPUT_ANALOG:
            raise IndexError("Pin 3 is NOT an ANALOG input! Use something else!")

        data = 0b000

        if mode == INPUT:
            data = 0b001
        if mode == INPUT_PULLUP:
            data = 0b101
        if mode == INPUT_ANALOG:
            data = 0b011

        self._bus.write_byte_data(B_I2C_ADR, B_I2C_GPIO_CONTROL_START + pin - 1, data)

    def digital_read(self, pin):
        pin = self._pin_map[pin]
        return bool(self._bus.read_byte_data(B_I2C_ADR, B_I2C_GPIO_START + pin - 1))

    def analog_read(self, pin):
        pin = self._pin_map[pin]
        if pin == 2:
            raise IndexError("Pin 3 is NOT an ANALOG input! Use something else!")

        # command = B_I2C_GPIO_ANALOG_START_L + (2 * (pin - 1))
        command = B_I2C_GPIO_ANALOG_START_L
        if pin == 3:
            command = B_I2C_GPIO_ANALOG_START_L + 2
        if pin == 4:
            command = B_I2C_GPIO_ANALOG_START_L + 4
        return self._bus.read_byte_data(B_I2C_ADR, command) + (self._bus.read_byte_data(B_I2C_ADR, command + 1) << 7)

    def digital_write(self, pin, data):
        pin = self._pin_map[pin]
        self._bus.write_byte_data(B_I2C_ADR, B_I2C_GPIO_START + pin - 1, int(data))
