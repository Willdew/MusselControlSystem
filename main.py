from backend import *
import machine

# Pin definitions
peltier_pin = 25
fan_pin = 18
cooler_step = 15
cooler_dir = 33
thermometer_pin = 32
feed_motor_dir = 27
feed_motor_step = 12
solenoid_recirculate = 26
solenoid_feed = 21
od_pin = 34
led_pin = 13
feed_per_mussel_per_30_min = 500
mussel_amount = 5
leak_pin = 14

# Initialize objects for main instance constructors
od_sensor = ODSensor(od_pin)
led = LED(led_pin)
feeder_motor = DirectionalMotorControl(feed_motor_dir, feed_motor_step, solenoid_recirculate, solenoid_feed)
thermometer = Thermometer(thermometer_pin)
peltier = PeltierControl(peltier_pin, fan_pin, cooler_dir, cooler_step)
client = MQTTClient(user = "scottienoy", password = "aio_UmYX32orwHHY8Nwa6pKJCORY2or4", server = "io.adafruit.com", client_id = "fdd1bd8f-edf6-4382-be10-c3e4466ed935")

# Initialize the main objects
cooler = Pid(thermometer, peltier, 20, 0.1, 1, 20.8, 1)
feeder = AlgaeFeeder(od_sensor, led, feeder_motor, feed_per_mussel_per_30_min, mussel_amount)
web_handler = Client(client, cooler, feeder)

cooler.init_PID()
feeder.init_feeder()
web_handler.init_client()

# Interrupt for leak detection
def leak_callback(p):
    web_handler.leak()

leak = pin = machine.Pin(leak_pin, machine.Pin.IN)
leak.irq(trigger=machine.Pin.IRQ_FALLING, handler=leak_callback)


