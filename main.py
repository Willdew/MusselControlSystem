from MusselMemory import *

# Pin definitions
peltier_pin = 66
cooler_step = 66
cooler_dir = 66
thermometer_pin = 66
feed_motor_dir = 66
feed_motor_step = 66
solenoid_recirculate = 66
solenoid_feed = 66
od_pin = 66
feed_per_mussel_per_30_min = 66
mussel_amount = 66

# Initialize objects for main instance constructors
od_sensor = ODSensor(od_pin)
feeder_motor = DirectionalMotorControl(feed_motor_dir, feed_motor_step, solenoid_recirculate, solenoid_feed)
thermometer = Thermometer(thermometer_pin)
peltier = PeltierControl(peltier_pin, cooler_dir, cooler_step)

# Initialize the main objects
cooler = Pid(thermometer, peltier, 25, 10, 10, 19, 5)
feeder = AlgaeFeeder(od_sensor, feeder_motor, feed_per_mussel_per_30_min, mussel_amount)

cooler.init_PID()
feeder.init_feeder()
