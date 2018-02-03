#!/usr/bin/env python
# encoding: utf-8

import ThunderBorg

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


class ThunderBorgMotorChannel(object):
    def __init__(self, tb, index):
        self._tb = tb
        self._index = index

    @property
    def power(self):
        if self._index == 0:
            return self._tb.GetMotor1() * 100.0
        else:
            return self._tb.GetMotor2() * 100.0

    @power.setter
    def power(self, value):
        if self._index == 0:
            self._tb.SetMotor1(float(value) / 100.0)
        else:
            self._tb.SetMotor2(float(value) / 100.0)


class ThunderBorgLED(object):
    def __init__(self, tb):
        self._tb = tb

        if self._tb.foundChip:
            self._tb.SetLedShowBattery(False)
            self._tb.SetLed1(1.0, 1.0, 1.0)

    @property
    def colour(self):
        return self._tb.GetLed1()

    @colour.setter
    def colour(self, value):
        self._tb.SetLed1(float(value[0]) / 255.0, float(value[1]) / 255.0, float(value[2]) / 255.0)

    def off(self):
        self._tb.SetLedShowBattery(True)


class ThunderBorgBoard(object):
    def __init__(self, address=ThunderBorg.I2C_ID_THUNDERBORG):
        self._tb = ThunderBorg.ThunderBorg()
        self._tb.i2cAddress = address
        self._tb.Init()

        if not self._tb.foundChip:
            raise Exception("No ThunderBorg at {}".format(address))

        self.m0 = ThunderBorgMotorChannel(self._tb, 0)
        self.m1 = ThunderBorgMotorChannel(self._tb, 1)
        self.led = ThunderBorgLED(self._tb)

    def off(self):
        if self._tb.foundChip:
            print "Switching ThunderBorg off at %02X" % self._tb.i2cAddress
            self._tb.MotorsOff()
            self.led.off()


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
