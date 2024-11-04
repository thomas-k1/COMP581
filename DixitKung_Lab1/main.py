#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

ultrasonic_sensor = UltrasonicSensor(Port.S1)
bump_sensor = TouchSensor(Port.S4)

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Write your program here.

##objective1
# init = (ultrasonic_sensor.distance()) / 10

# curr = (ultrasonic_sensor.distance()) / 10
# while curr > init - 120:
#     left_motor.run(250)
#     right_motor.run(250)
#     curr = (ultrasonic_sensor.distance()) / 10


wheel_diameter = 5.6
wheel_circum = (3.14159265358979) * wheel_diameter

rotations = 120 / wheel_circum
left_motor.run_angle(250, rotations * 360, wait=False)
right_motor.run_angle(250, rotations * 360)


ev3.speaker.beep()
curr = (ultrasonic_sensor.distance()) / 10
# left_motor.stop()
# right_motor.stop()


##objective 2
while True:
    if Button.CENTER in ev3.buttons.pressed():

        while (curr > 50):
            left_motor.run(250)
            right_motor.run(250)
            curr = (ultrasonic_sensor.distance()) / 10

        ev3.speaker.beep()
        left_motor.stop()
        right_motor.stop()
        break

##objective 3
while True:
    if Button.CENTER in ev3.buttons.pressed():
        while not bump_sensor.pressed():
            left_motor.run(250)
            right_motor.run(250)
        
        left_motor.stop()
        right_motor.stop()
        curr = (ultrasonic_sensor.distance()) / 10
        while (curr < 50):
            left_motor.run(-250)
            right_motor.run(-250)
            curr = (ultrasonic_sensor.distance()) / 10
        
        ev3.speaker.beep()
        left_motor.stop()
        right_motor.stop()
        break


