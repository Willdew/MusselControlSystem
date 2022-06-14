# This file contains all classes and functions for the MusselControl program.

# importing libraries
import time
from machine import *
import _thread
import time
import machine
import statistics
from math import log
from simple_pid import PID


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

    # Description setter for steps per milliliter
    # input: steps_per_ml(int)
    # output: none
    def set_steps_per_ml(self, steps_per_ml: int):
        self.__steps_per_ml = steps_per_ml

    # Description: internal method for moving the motor
    # input: steps(int), direction(string)
    # output: none
    def __step(self, steps: int, direction: str):
        self.__MutEx.acquire()
        if direction == "forward":
            self.__dir_pin.on()
        elif direction == "backward":
            self.__dir_pin.off()
        for _ in range(steps):
            self.__step_pin.on()
            time.sleep(0.001)
            self.__step_pin.off()
            time.sleep(0.001)
        self.__MutEx.release()

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


class thermometer:
    def __init__(self, pin):
        self.__adc_V_lookup = [0.02470588, 0.02058824, 0.04117647, 0.06176471, 0.06588235, 0.07, 0.07411765, 0.07720589,
                               0.08029412, 0.08338236, 0.08647059, 0.08955883, 0.09264707, 0.0957353, 0.09882354, 0.105,
                               0.1111765, 0.1142647, 0.117353, 0.1204412, 0.1235294, 0.126, 0.1284706, 0.1309412,
                               0.1334118,
                               0.1358824, 0.14, 0.1441177, 0.1482353, 0.1513235, 0.1544118, 0.1575, 0.1605882,
                               0.1647059,
                               0.1688235, 0.1729412, 0.1760294, 0.1791177, 0.1822059, 0.1852941, 0.1883824, 0.1914706,
                               0.1945588, 0.1976471, 0.2007353, 0.2038235, 0.2069118, 0.21, 0.2141177, 0.2182353,
                               0.222353,
                               0.2264706, 0.2305882, 0.2347059, 0.2377941, 0.2408824, 0.2439706, 0.2470588, 0.2501471,
                               0.2532353, 0.2563236, 0.2594118, 0.2625, 0.2655883, 0.2686765, 0.2717647, 0.2758824,
                               0.28,
                               0.2841177, 0.2872059, 0.2902942, 0.2933824, 0.2964706, 0.2995588, 0.3026471, 0.3057353,
                               0.3088235, 0.3119118, 0.315, 0.3180882, 0.3211765, 0.3252941, 0.3294118, 0.3335294,
                               0.3366177,
                               0.3397059, 0.3427941, 0.3458824, 0.3489706, 0.3520588, 0.3551471, 0.3582353, 0.362353,
                               0.3664706, 0.3705883, 0.3736765, 0.3767647, 0.379853, 0.3829412, 0.3854118, 0.3878823,
                               0.390353,
                               0.3928235, 0.3952941, 0.3994118, 0.4035295, 0.4076471, 0.4117647, 0.4158824, 0.42,
                               0.4230883,
                               0.4261765, 0.4292647, 0.432353, 0.4354412, 0.4385294, 0.4416177, 0.4447059, 0.4488235,
                               0.4529412, 0.4570589, 0.4601471, 0.4632353, 0.4663236, 0.4694118, 0.4725, 0.4755883,
                               0.4786765,
                               0.4817647, 0.484853, 0.4879412, 0.4910295, 0.4941177, 0.4965883, 0.4990589, 0.5015294,
                               0.504,
                               0.5064706, 0.5105882, 0.5147059, 0.5188236, 0.5219118, 0.525, 0.5280883, 0.5311765,
                               0.5342648,
                               0.537353, 0.5404412, 0.5435295, 0.5466177, 0.5497059, 0.5527942, 0.5558824, 0.5620589,
                               0.5682353, 0.5713236, 0.5744118, 0.5775001, 0.5805883, 0.5836765, 0.5867648, 0.589853,
                               0.5929412, 0.5970588, 0.6011765, 0.6052941, 0.6077647, 0.6102353, 0.6127059, 0.6151765,
                               0.6176471, 0.6217647, 0.6258824, 0.63, 0.6330882, 0.6361765, 0.6392647, 0.642353,
                               0.6464706,
                               0.6505883, 0.6547059, 0.6588235, 0.6629412, 0.6670588, 0.6695294, 0.672, 0.6744706,
                               0.6769412,
                               0.6794118, 0.6835294, 0.6876471, 0.6917647, 0.6948529, 0.6979412, 0.7010294, 0.7041177,
                               0.7072059, 0.7102942, 0.7133823, 0.7164706, 0.7195588, 0.7226471, 0.7257353, 0.7288236,
                               0.7329412, 0.7370589, 0.7411765, 0.7442647, 0.747353, 0.7504412, 0.7535295, 0.7566176,
                               0.7597059, 0.7627941, 0.7658824, 0.7689706, 0.7720589, 0.7751471, 0.7782353, 0.7905883,
                               0.7936765, 0.7967648, 0.7998529, 0.8029412, 0.8060294, 0.8091177, 0.8122059, 0.8152942,
                               0.8194118, 0.8235294, 0.8276471, 0.8307353, 0.8338236, 0.8369118, 0.8400001, 0.8441177,
                               0.8482353, 0.852353, 0.8554412, 0.8585295, 0.8616177, 0.8647059, 0.8677941, 0.8708824,
                               0.8739706, 0.8770589, 0.8795294, 0.8820001, 0.8844706, 0.8869412, 0.8894118, 0.8935295,
                               0.8976471, 0.9017648, 0.9042353, 0.906706, 0.9091764, 0.9116471, 0.9141177, 0.9172059,
                               0.9202942, 0.9233824, 0.9264707, 0.9305883, 0.9347058, 0.9388236, 0.9419118, 0.9450001,
                               0.9480883, 0.9511765, 0.9542647, 0.957353, 0.9604412, 0.9635295, 0.9676472, 0.9717647,
                               0.9758824, 0.9789706, 0.9820589, 0.9851471, 0.9882354, 0.9913236, 0.9944118, 0.9975,
                               1.000588,
                               1.004706, 1.008824, 1.012941, 1.015412, 1.017882, 1.020353, 1.022824, 1.025294, 1.029412,
                               1.033529, 1.037647, 1.040735, 1.043824, 1.046912, 1.05, 1.053088, 1.056177, 1.059265,
                               1.062353,
                               1.066471, 1.070588, 1.074706, 1.077176, 1.079647, 1.082118, 1.084588, 1.087059, 1.091177,
                               1.095294, 1.099412, 1.103529, 1.107647, 1.111765, 1.114853, 1.117941, 1.121029, 1.124118,
                               1.127206, 1.130294, 1.133382, 1.136471, 1.139559, 1.142647, 1.145735, 1.148824, 1.152941,
                               1.157059, 1.161177, 1.163647, 1.166118, 1.168588, 1.171059, 1.17353, 1.177647, 1.181765,
                               1.185882, 1.188353, 1.190824, 1.193294, 1.195765, 1.198235, 1.201324, 1.204412, 1.2075,
                               1.210588, 1.214706, 1.218824, 1.222941, 1.226029, 1.229118, 1.232206, 1.235294, 1.239412,
                               1.243529, 1.247647, 1.250118, 1.252588, 1.255059, 1.257529, 1.26, 1.264118, 1.268235,
                               1.272353,
                               1.276471, 1.280588, 1.284706, 1.287794, 1.290882, 1.293971, 1.297059, 1.300147, 1.303235,
                               1.306324, 1.309412, 1.3125, 1.315588, 1.318676, 1.321765, 1.324853, 1.327941, 1.331029,
                               1.334118, 1.336588, 1.339059, 1.341529, 1.344, 1.346471, 1.350588, 1.354706, 1.358824,
                               1.361912,
                               1.365, 1.368088, 1.371176, 1.374265, 1.377353, 1.380441, 1.383529, 1.387647, 1.391765,
                               1.395882,
                               1.398971, 1.402059, 1.405147, 1.408235, 1.411324, 1.414412, 1.4175, 1.420588, 1.424706,
                               1.428824, 1.432941, 1.436029, 1.439118, 1.442206, 1.445294, 1.448382, 1.451471, 1.454559,
                               1.457647, 1.460735, 1.463824, 1.466912, 1.47, 1.473088, 1.476177, 1.479265, 1.482353,
                               1.486471,
                               1.490588, 1.494706, 1.497177, 1.499647, 1.502118, 1.504588, 1.507059, 1.511177, 1.515294,
                               1.519412, 1.52353, 1.527647, 1.531765, 1.534235, 1.536706, 1.539176, 1.541647, 1.544118,
                               1.548235, 1.552353, 1.556471, 1.559559, 1.562647, 1.565735, 1.568824, 1.570196, 1.571569,
                               1.572941, 1.574314, 1.575686, 1.577059, 1.578432, 1.579804, 1.581177, 1.584265, 1.587353,
                               1.590441, 1.593529, 1.596618, 1.599706, 1.602794, 1.605882, 1.608971, 1.612059, 1.615147,
                               1.618235, 1.621324, 1.624412, 1.6275, 1.630588, 1.634706, 1.638824, 1.642941, 1.646029,
                               1.649118, 1.652206, 1.655294, 1.659412, 1.663529, 1.667647, 1.670735, 1.673824, 1.676912,
                               1.68,
                               1.684118, 1.688235, 1.692353, 1.694824, 1.697294, 1.699765, 1.702235, 1.704706, 1.708824,
                               1.712941, 1.717059, 1.720147, 1.723235, 1.726324, 1.729412, 1.7325, 1.735588, 1.738677,
                               1.741765, 1.744235, 1.746706, 1.749177, 1.751647, 1.754118, 1.758235, 1.762353, 1.766471,
                               1.769559, 1.772647, 1.775735, 1.778824, 1.781912, 1.785, 1.788088, 1.791177, 1.794265,
                               1.797353,
                               1.800441, 1.80353, 1.807647, 1.811765, 1.815882, 1.82, 1.824118, 1.828235, 1.831324,
                               1.834412,
                               1.8375, 1.840588, 1.843677, 1.846765, 1.849853, 1.852941, 1.857059, 1.861177, 1.865294,
                               1.869412, 1.87353, 1.877647, 1.880118, 1.882588, 1.885059, 1.887529, 1.89, 1.896177,
                               1.902353,
                               1.904823, 1.907294, 1.909765, 1.912235, 1.914706, 1.917794, 1.920882, 1.923971, 1.927059,
                               1.931176, 1.935294, 1.939412, 1.9425, 1.945588, 1.948677, 1.951765, 1.954853, 1.957941,
                               1.96103,
                               1.964118, 1.966588, 1.969059, 1.97153, 1.974, 1.976471, 1.980588, 1.984706, 1.988824,
                               1.991912,
                               1.995, 1.998088, 2.001177, 2.005294, 2.009412, 2.01353, 2.016618, 2.019706, 2.022794,
                               2.025882,
                               2.03, 2.034118, 2.038235, 2.042353, 2.046471, 2.050588, 2.053677, 2.056765, 2.059853,
                               2.062941,
                               2.06603, 2.069118, 2.072206, 2.075294, 2.078382, 2.081471, 2.084559, 2.087647, 2.090735,
                               2.093824, 2.096912, 2.1, 2.103088, 2.106177, 2.109265, 2.112353, 2.115441, 2.11853,
                               2.121618,
                               2.124706, 2.128824, 2.132941, 2.137059, 2.140147, 2.143235, 2.146324, 2.149412, 2.1525,
                               2.155588, 2.158677, 2.161765, 2.164235, 2.166706, 2.169177, 2.171647, 2.174118, 2.178235,
                               2.182353, 2.186471, 2.189559, 2.192647, 2.195735, 2.198824, 2.201912, 2.205, 2.208088,
                               2.211177,
                               2.215294, 2.219412, 2.22353, 2.226, 2.228471, 2.230941, 2.233412, 2.235883, 2.24,
                               2.244118,
                               2.248235, 2.251324, 2.254412, 2.2575, 2.260588, 2.266765, 2.272941, 2.275412, 2.277882,
                               2.280353, 2.282824, 2.285294, 2.288383, 2.291471, 2.294559, 2.297647, 2.300735, 2.303824,
                               2.306912, 2.31, 2.313088, 2.316177, 2.319265, 2.322353, 2.325441, 2.32853, 2.331618,
                               2.334706,
                               2.338824, 2.342941, 2.347059, 2.350147, 2.353235, 2.356324, 2.359412, 2.371765, 2.373824,
                               2.375882, 2.377941, 2.38, 2.382059, 2.384118, 2.388235, 2.392353, 2.396471, 2.399559,
                               2.402647,
                               2.405735, 2.408823, 2.411912, 2.415, 2.418088, 2.421176, 2.425294, 2.429412, 2.433529,
                               2.436618,
                               2.439706, 2.442794, 2.445882, 2.448971, 2.452059, 2.455147, 2.458235, 2.461323, 2.464412,
                               2.4675, 2.470588, 2.473676, 2.476765, 2.479853, 2.482941, 2.486029, 2.489118, 2.492206,
                               2.495294, 2.498382, 2.501471, 2.504559, 2.507647, 2.510735, 2.513824, 2.516912, 2.52,
                               2.523088,
                               2.526176, 2.529265, 2.532353, 2.534824, 2.537294, 2.539765, 2.542235, 2.544706, 2.548824,
                               2.552941, 2.557059, 2.559118, 2.561177, 2.563235, 2.565294, 2.567353, 2.569412, 2.5725,
                               2.575588, 2.578676, 2.581765, 2.584853, 2.587941, 2.591029, 2.594118, 2.597206, 2.600294,
                               2.603382, 2.606471, 2.609559, 2.612647, 2.615735, 2.618824, 2.621294, 2.623765, 2.626235,
                               2.628706, 2.631176, 2.633647, 2.636118, 2.638588, 2.641059, 2.643529, 2.646618, 2.649706,
                               2.652794, 2.655882, 2.658971, 2.662059, 2.665147, 2.668235, 2.670294, 2.672353, 2.674412,
                               2.676471, 2.67853, 2.680588, 2.686765, 2.692941, 2.695, 2.697059, 2.699118, 2.701177,
                               2.703235,
                               2.705294, 2.707765, 2.710235, 2.712706, 2.715177, 2.717647, 2.720735, 2.723824, 2.726912,
                               2.73,
                               2.733088, 2.736176, 2.739265, 2.742353, 2.744118, 2.745883, 2.747647, 2.749412, 2.751176,
                               2.752941, 2.754706, 2.756765, 2.758824, 2.760882, 2.762941, 2.765, 2.767059, 2.770147,
                               2.773235,
                               2.776324, 2.779412, 2.781882, 2.784353, 2.786824, 2.789294, 2.791765, 2.794235, 2.796706,
                               2.799177, 2.801647, 2.804118, 2.805882, 2.807647, 2.809412, 2.811176, 2.812941, 2.814706,
                               2.816471, 2.818941, 2.821412, 2.823883, 2.826353, 2.828824, 2.831912, 2.835, 2.838088,
                               2.841177,
                               2.843235, 2.845294, 2.847353, 2.849412, 2.851471, 2.853529, 2.855588, 2.857647, 2.859706,
                               2.861765, 2.863824, 2.865882, 2.867941, 2.87, 2.872059, 2.874118, 2.876177, 2.878235,
                               2.880294,
                               2.882353, 2.884412, 2.886471, 2.88853, 2.890588, 2.892353, 2.894118, 2.895882, 2.897647,
                               2.899412, 2.901177, 2.902941, 2.905, 2.907059, 2.909118, 2.911177, 2.913235, 2.915294,
                               2.918382,
                               2.921471, 2.924559, 2.927647, 2.929412, 2.931177, 2.932941, 2.934706, 2.936471, 2.938235,
                               2.94,
                               2.941765, 2.94353, 2.945294, 2.947059, 2.948823, 2.950588, 2.952353, 2.954412, 2.956471,
                               2.958529, 2.960588, 2.962647, 2.964706, 2.966765, 2.968824, 2.970882, 2.972941, 2.975,
                               2.977059,
                               2.978824, 2.980588, 2.982353, 2.984118, 2.985882, 2.987647, 2.989412, 2.991177, 2.992941,
                               2.994706, 2.996471, 2.998235, 3.0, 3.001765, 3.00353, 3.005294, 3.007059, 3.008824,
                               3.010588,
                               3.012353, 3.014118, 3.016588, 3.019059, 3.02153, 3.024, 3.026471, 3.027843, 3.029216,
                               3.030588,
                               3.031961, 3.033334, 3.034706, 3.036078, 3.037451, 3.038824, 3.041294, 3.043765, 3.046236,
                               3.048706, 3.051177, 3.052721, 3.054265, 3.055809, 3.057353, 3.058897, 3.060441, 3.061985,
                               3.063529, 3.065294, 3.067059, 3.068824, 3.070588, 3.072353, 3.074118, 3.075882, 3.077647,
                               3.079412, 3.081177, 3.082941, 3.084706, 3.086471, 3.088235, 3.090294, 3.092353, 3.094412,
                               3.096471, 3.098529, 3.100588, 3.102647, 3.104706, 3.106765, 3.108824, 3.110882, 3.112941,
                               3.114706, 3.116471, 3.118235, 3.12, 3.121765, 3.12353, 3.137647]

        self.__NOM_RES = 10000
        self.__SER_RES = 9820
        self.__TEMP_NOM = 25
        self.__NUM_SAMPLES = 25
        self.__THERM_B_COEFF = 3950
        self.__ADC_MAX = 1023
        self.__ADC_Vmax = 3.15
        self.__adc = ADC(Pin(pin))
        self.__adc.atten(ADC.ATTN_11DB)
        self.__adc.width(ADC.WIDTH_10BIT)

    def read_temp(self):
        raw_read = []
        # Collect NUM_SAMPLES
        for i in range(1, self.__NUM_SAMPLES + 1):
            raw_read.append(self.__adc.read())

        # Average of the NUM_SAMPLES and look it up in the table
        raw_average = sum(raw_read) / self.__NUM_SAMPLES
        print('raw_avg = ' + str(raw_average))
        print('V_measured = ' + str(self.__adc_V_lookup[round(raw_average)]))

        # Convert to resistance
        raw_average = self.__ADC_MAX * self.__adc_V_lookup[round(raw_average)] / self.__ADC_Vmax
        resistance = (self.__SER_RES * raw_average) / (self.__ADC_MAX - raw_average)
        print('Thermistor resistance: {} ohms'.format(resistance))

        # Convert to temperature
        steinhart = log(resistance / self.__NOM_RES) / self.__THERM_B_COEFF
        steinhart += 1.0 / (self.__TEMP_NOM + 273.15)
        steinhart = (1.0 / steinhart) - 273.15
        return steinhart


# Description: This class is able to control the cooling system
class Pid:
    def __init__(self, temp_sensor: thermometer, peltier_element, kp, ki, kd, set_temp, timer):
        self.__temp_sensor = temp_sensor
        self.__peltier_element = peltier_element
        self.__Kp = kp
        self.__Ki = ki
        self.__Kd = kd
        self.__set_temp = set_temp
        self.__timer = timer
        self.__pid = PID(self.__Kp, self.__Ki, self.__Kd, setpoint=self.__set_temp)

    def set_temp(self, temp):
        self.__set_temp = temp

    def get_set_temp(self):
        return self.__set_temp

    def get_temp(self):
        return self.__temp_sensor.read_temp()

    # note: since the PID is used in reverse mode, all values are negative
    def set_pid(self, kp, ki, kd):
        self.__Kp = kp
        self.__Ki = ki
        self.__Kd = kd

    def get_pid(self):
        return self.__Kp, self.__Ki, self.__Kd

    def get_timer(self):
        return self.__timer

    # period: 1000 = 1 second
    def pid_timer(self):
        tim0 = machine.Timer(0)
        tim0.init(period=self.__timer * 1000, mode=machine.Timer.PERIODIC, callback=lambda t: update_pid())

    # Description: this method runs the PID algorithm and turns the cooling system on or off
    def update_pid(self):
        output = self.__pid(self.get_temp())
        self.__pid.output_limits(0, 10)

        if output > 3:
            self.__peltier_element.cooling_on()
        else:
            self.__peltier_element.cooling_off()