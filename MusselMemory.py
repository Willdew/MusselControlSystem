#This file contains all classes and functions for the MusselControl program.
import os
import time
import machine
import statistics

class ODSensor:
    # Constructor
    def __init__(self, pin):
        self.__pin = pin
        self.PD = machine.ADC(machine.Pin(self.__pin))
        self.PD.atten(machine.ADC.ATTN_11DB)
        self.PD.width(machine.ADC.WIDTH_10BIT)

    #Getter-method for pin
    def get_pin(self):
        return self.__pin

    def measure_OD(self):
        big = []
        for _ in range(20):
            big.append(self.PD.read)
        a = statistics.mean(big)
        return a

class LED:
    def __init__(self,pin):
        self.__pin = pin
        self.pwn = machine.ADC(machine.Pin(self.__pin))
        self.pwm.duty(52)
        self.pwm.freq(78000)

    #getter-method for pin
    def get_pin(self):
        return self.__pin

    #setter for freq
    def set_freq(self, f):
        self.pwn.freq(f)

    #setter for duty
    def set_duty(self, d):
        self.pwn.duty(d)