# This file contains all classes and functions for the MusselControl program.
import os
import time
import machine
import statistics


# Description: This class is able to read the data of an OD sensor
class ODSensor:
    # Constructor
    # input: pin(int)
    # output: none
    def __init__(self, pin):
        self.__pin = pin
        self.PD = machine.ADC(machine.Pin(self.__pin))
        self.PD.atten(machine.ADC.ATTN_11DB)
        self.PD.width(machine.ADC.WIDTH_10BIT)

    # Description: This method gets the value of the pin of the sensor
    # input: none
    # output: value(int)
    def get_pin(self):
        return self.__pin

    # Description: This method gets the measured value of the sensor
    # input: none
    # output: value(int)
    def measure_OD(self):
        big = []
        for _ in range(20):
            big.append(self.PD.read)
        a = statistics.mean(big)
        return a


# Description: This class is able to control an LED
class LED:
    # Constructor
    # input: pin(int)
    # output: none
    def __init__(self, pin):
        self.__pin = pin
        self.__LED = machine.Pin(self.__pin, machine.Pin.OUT)

    # Description: This method gets the value of the pin of the sensor
    # input: none
    # output: value(int)
    def get_pin(self):
        return self.__pin

    # Description: This method turns the LED on
    # input: none
    # output: none
    def on(self):
        self.__LED.value(1)

    # Description: This method turns the LED off
    # input: none
    # output: none
    def off(self):
        self.__LED.value(0)
