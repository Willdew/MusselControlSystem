import os
import time
import machine
import statistics

class od_sensor:

    def __init__(self, pin):
        self.__pin = pin
        self.__PD = machine.ADC(machine.Pin(self.pin))
        self.PD.atten(machine.ADC.ATTN_11DB)
        self.PD.width(machine.ADC.WIDTH_10BIT)

    def read_od(self):
        big = []
        for _ in range(20):
            big.append(self.PD.read())

        return statistics.mean(big)


class led_control:
    def __init__(self, pin):
        self.__pin = pin
        self.__pwn = machine.PWN(machine.Pin(self.__pin))
        self.pwn.duty(52)
        self.pwn.freq(78000)

    def set_duty(self,d):
        self.pwn.duty(d)

    def set_freq(self,f):
        self.pwn.freq(f)