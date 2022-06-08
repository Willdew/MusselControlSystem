# This file contains all classes and functions for the MusselControl program.

# importing libraries
import time
from machine import *


# Description: base class for controlling the A4988 stepper motor controller
class MotorControl:

    # Description: Constructor
    # input: dir_pin(int), step_pin(int)
    # output: none
    def __init__(self, dir_pin, step_pin):
        self.__dir_pin = Pin(dir_pin, Pin.OUT)
        self.__step_pin = Pin(step_pin, Pin.OUT)

    # Description: Method for moving the motor
    # input: steps(int), direction(string)
    # output: none
    def step(self, steps: int, direction: str):
        if direction == "forward":
            self.__dir_pin.on()
        elif direction == "backward":
            self.__dir_pin.off()
        for pik in range(steps):
            self.__step_pin.on()
            time.sleep(0.001)
            self.__step_pin.off()
            time.sleep(0.001)

    # Description: Method for moving a certain amount of milliliter
    # input: ml(int), direction(string)
    # output: none
    def step_ml(self, ml: int, direction: str):
        steps = ml*300
        self.step(steps, direction)


