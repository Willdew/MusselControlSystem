# This file contains all classes and functions for the MusselControl program.

# importing libraries
import time
from machine import *
import _thread
import time
import machine
import statistics


# Description: base class for controlling the A4988 stepper motor controller
class MotorControl:

    # Description: Constructor
    # input: dir_pin(int), step_pin(int)
    # output: none
    def __init__(self, dir_pin, step_pin):
        self.__dir_pin = Pin(dir_pin, Pin.OUT)
        self.__step_pin = Pin(step_pin, Pin.OUT)
        self.__MutEx = _thread.allocate_lock()
        self.__steps_per_ml = 300


# Description: Class for controlling a motor with solenoid valves
class DirectionalMotorControl(MotorControl):

    # Description: Constructor
    # input: dir_pin(int), step_pin(int), sol_pin1:(int), sol_pin2:(int)
    # output: none
    def __init__(self, dir_pin, step_pin, sol_pin1, sol_pin2):
        super().__init__(dir_pin, step_pin)
        self.__sol_pin1 = LED(sol_pin1)
        self.__sol_pin2 = LED(sol_pin2)

    # Description setter for steps per milliliter
    # input: steps_per_ml(int)
    # output: none
    def set_steps_per_ml(self, steps_per_ml: int):
        self.__steps_per_ml = steps_per_ml

    # Description: internal method for moving the motor
    # input: steps(int), direction(string)
    # output: none
    def __step(self, steps: int, direction: str, valve: int):
        self.__MutEx.acquire()
        if valve == 1:
            self.__sol_pin1.on()
            self.__sol_pin2.off()
        elif valve == 2:
            self.__sol_pin1.off()
            self.__sol_pin2.on()
        else:
            steps = 0
        if direction == "forward":
            self.__dir_pin.on()
        elif direction == "backward":
            self.__dir_pin.off()
        for _ in range(steps):
            self.__step_pin.on()
            time.sleep(0.001)
            self.__step_pin.off()
            time.sleep(0.001)
        self.__sol_pin1.off()
        self.__sol_pin2.off()
        self.__MutEx.release()

    # Description: method for starting thread for moving the motor
    # input: steps(int), direction(string)
    # output: none
    def step(self, steps: int, direction: str, valve: int):
        _thread.start_new_thread(self.__step, (steps, direction, valve))

    # Description: Method for moving a certain amount of milliliter
    # input: ml(int), direction(string)
    # output: none
    def step_ml(self, ml: int, direction: str, valve: int):
        steps = ml * self.__steps_per_ml
        self.step(steps, direction, valve)


# Description: Class for controlling the stepper motor with pwn ability
# extends MotorControl
class PwmMotorControl(MotorControl):

    # Description: Constructor
    # input: dir_pin(int), step_pin(int)
    # output: none
    def __init__(self, dir_pin, step_pin):
        super().__init__(dir_pin, step_pin)
        self.__motor_speed = 1000
        self.__pwm_pin = Pin(step_pin, Pin.OUT)
        self.__pwm = PWM(self.__pwm_pin, freq=self.__motor_speed, duty=0)

    # Description: Method for setting the speed of the motor
    # input: speed(int)
    # output: none
    def set_speed(self, speed: int):
        self.__motor_speed = speed
        self.__pwm.freq(speed)

    # Description: Method for turning on the motor
    # input: direction(string)
    # output: none
    def motor_on(self, direction: str):
        if direction == "forward":
            self.__dir_pin.on()
        elif direction == "backward":
            self.__dir_pin.off()
        self.__pwm.duty(50)

    # Description: Method for turning off the motor
    # input: none
    # output: none
    def motor_off(self):
        self.__pwm.duty(0)


# Description: Class for controlling the cooling system
class PeltierControl:
    # Description: Constructor
    # input: fan_pin(int), peltier_pin(int), dir_pin(int), step_pin(int)
    # output: none
    def __init__(self, fan_pin, peltier_pin, dir_pin, step_pin):
        self.__motor = PwmMotorControl(dir_pin, step_pin)
        self.__fan_pin = Pin(fan_pin, Pin.OUT)
        self.__peltier_pin = Pin(peltier_pin, Pin.OUT)

    # Description: Method for turning on the cooling system
    # input: none
    # output: none
    def cooling_on(self):
        self.__fan_pin.on()
        self.__peltier_pin.on()
        self.__motor.motor_on("forward")

    # Description: Method for turning off the cooling system
    # input: none
    # output: none
    def cooling_off(self):
        self.__fan_pin.off()
        self.__peltier_pin.off()
        self.__motor.motor_off()


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
