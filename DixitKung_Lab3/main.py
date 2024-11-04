#lab 3

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile

from utime import sleep


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def buttonPress():
    if bump_sensor.pressed():
        rotations = 15 / wheel_circum
        left_motor.run_angle(-200, rotations * 360, wait=False)
        right_motor.run_angle(-200, rotations * 360)

        left_motor.run_angle(250, 0.75 * 360, wait=False)
        right_motor.run_angle(-250, 0.75 * 360)

# Create your objects here.
ev3 = EV3Brick()

ultrasonic_sensor = UltrasonicSensor(Port.S1)
bump_sensor = TouchSensor(Port.S4)

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)


wheel_diameter = 5.6
wheel_circum = (3.14159265358979) * wheel_diameter

# Objective 1
while not bump_sensor.pressed():
    left_motor.run(200)
    right_motor.run(200)
    
left_motor.stop()
right_motor.stop()

rotations = 15 / wheel_circum
left_motor.run_angle(-200, rotations * 360, wait=False)
right_motor.run_angle(-200, rotations * 360)

left_motor.run_angle(250, 0.5 * 360, wait=False)
right_motor.run_angle(-250, 0.5 * 360)
#right_motor.run_angle(-250, 0.5 * 360)

# Objective 2

adjustments = 0.3 * 360

current_distance = (ultrasonic_sensor.distance()) / 10
if current_distance < 10:
    right_motor.run_angle(75, adjustments, wait=False)
    left_motor.run_angle(100, adjustments)
    right_motor.brake()
elif current_distance > 20:
    left_motor.run_angle(75, adjustments, wait=False)
    right_motor.run_angle(100, adjustments)
    left_motor.brake()
else:
    left_motor.run_angle(100, adjustments, wait=False)
    right_motor.run_angle(100, adjustments)

new_distance = (ultrasonic_sensor.distance()) / 10

distance_traveled = 0
travel_distance = 205
while distance_traveled < travel_distance:
#while True:
    #diff = abs(new_distance - current_distance) / 2
    buttonPress()

    if new_distance < current_distance:
        right_motor.run_angle(75, adjustments, wait=False)
        left_motor.run_angle(100, adjustments)
        right_motor.brake()

    elif new_distance > current_distance:
        left_motor.run_angle(75, adjustments, wait=False)
        right_motor.run_angle(100, adjustments)
        left_motor.brake()
    else:
        left_motor.run_angle(100, adjustments, wait=False)
        right_motor.run_angle(100, adjustments)

    current_distance = new_distance
    new_distance = (ultrasonic_sensor.distance()) / 10

    if new_distance < 10:
        left_motor.run_angle(360, 0.5 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(-360, 0.5 * 360)
        left_motor.run_angle(300, 0.5 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(300, 0.5 * 360)
        left_motor.run_angle(-360, 0.5 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(360, 0.5 * 360)
    elif new_distance > 20:
        left_motor.run_angle(-360, 0.25 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(360, 0.25 * 360)
        if ((ultrasonic_sensor.distance()) / 10) >= new_distance:
            left_motor.run_angle(-360, 0.25 * 360, wait=False)
            buttonPress()
            right_motor.run_angle(360, 0.25 * 360)
            left_motor.run_angle(300, 0.5 * 360, wait=False)
            buttonPress()
            right_motor.run_angle(300, 0.5 * 360)
            left_motor.run_angle(360, 0.5 * 360, wait=False)
            buttonPress()
            right_motor.run_angle(-360, 0.5 * 360)

    distance_traveled += 0.26 * wheel_circum
    #sleep(0.2)



'''
while True:
    current_distance = (ultrasonic_sensor.distance()) / 10
    if current_distance < 10:
        left_motor.stop()
        right_motor.stop()
        left_motor.run_angle(250, 1 * 360, wait=False)
        closest = 255
        angle = 0
        while left_motor.speed() > 0:
            curr = (ultrasonic_sensor.distance()) / 10
            if curr < closest:
                closest = curr
                angle = left_motor.angle()
        left_motor.run_angle(250, angle)
    elif current_distance >= 20:
        left_motor.stop()
        right_motor.stop()
        left_motor.run_angle(250, 1 * 360, wait=False)
        closest = 255
        angle = 0
        while left_motor.speed() > 0:
            curr = (ultrasonic_sensor.distance()) / 10
            if curr < closest:
                closest = curr
                angle = left_motor.angle()
        left_motor.run_angle(250, angle)
    #elif current_distance < 20:
    left_motor.run(100)
    right_motor.run(100)

    sleep(1)
'''

ev3.speaker.beep()