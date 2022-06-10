# This file contains all classes and functions for the MusselControl program.

# importing libraries
import time
from machine import *
import _thread


# Description: base class for controlling the A4988 stepper motor controller
class MotorControl:

    # Description: Constructor
    # input: dir_pin(int), step_pin(int)
    # output: none
    def __init__(self, dir_pin, step_pin):
        self.__dir_pin = Pin(dir_pin, Pin.OUT)
        self.__step_pin = Pin(step_pin, Pin.OUT)
        self.MutEx = _thread.allocate_lock()
        self.__steps_per_ml = 300

    # Description setter for steps per milliliter
    # input: steps_per_ml(int)
    # output: none
    def set_steps_per_ml(self, steps_per_ml: int):
        self.__steps_per_ml = steps_per_ml

    # Description: internal method for moving the motor
    # input: steps(int), direction(string)
    # output: none
    def __step(self, steps: int, direction: str):
        self.MutEx.acquire()
        if direction == "forward":
            self.__dir_pin.on()
        elif direction == "backward":
            self.__dir_pin.off()
        for _ in range(steps):
            self.__step_pin.on()
            time.sleep(0.001)
            self.__step_pin.off()
            time.sleep(0.001)
        self.MutEx.release()

    # Description: method for starting thread for moving the motor
    # input: steps(int), direction(string)
    # output: none
    def step(self, steps: int, direction: str):
        _thread.start_new_thread(self.__step, (steps, direction))

    # Description: Method for moving a certain amount of milliliter
    # input: ml(int), direction(string)
    # output: none
    def step_ml(self, ml: int, direction: str):
        steps = ml * self.__steps_per_ml
        self.step(steps, direction)


