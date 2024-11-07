#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile

from utime import sleep
import time
from math import pi, sin, cos
#import numpy as np

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def buttonPress():
    global new_x, new_y, new_theta
    
    if bump_sensor.pressed():
        rotations = 15 / wheel_circum
        start_time = time.time()
        left_motor.run_angle(-200, rotations * 360, wait=False)
        right_motor.run_angle(-200, rotations * 360)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, -200, -200, wheel_radius, L)

        start_time = time.time()
        left_motor.run_angle(250, 0.75 * 360, wait=False)
        right_motor.run_angle(-250, 0.75 * 360)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 250, -250, wheel_radius, L)

        start_time = time.time()


# Create your objects here.
ev3 = EV3Brick()

ultrasonic_sensor = UltrasonicSensor(Port.S1)
bump_sensor = TouchSensor(Port.S4)

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)


wheel_diameter = 5.6
wheel_radius = 2.8
wheel_circum = (3.14159265358979) * wheel_diameter
L = 12

def calculate_pose(x, y, theta, dt, left_u, right_u, r, L):
    left_v = left_u * (pi/180) * r
    right_v = right_u * (pi/180) * r

    if (left_v == right_v):
        new_x = x + (left_v * cos(theta) * dt)
        new_y = y + (left_v * sin(theta) * dt)
        new_theta = theta
    else:
        R = (L/2) * ((left_v + right_v) / (right_v - left_v))
        omega = (right_v - left_v) / L
        ICCx = x - (R * sin(theta))
        ICCy = y + (R * cos(theta))

        # rotation_matrix = np.array([
        #     [cos(omega*dt), -sin(omega*dt), 0],
        #     [sin(omega*dt), cos(omega*dt), 0],
        #     [0, 0, 1]
        # ])
        # position_matrix = np.array([[x-ICCx], [y-ICCy], [theta]])
        # addition_matrix = np.array([[ICCx], [ICCy], [omega*dt]])

        # new_pose = np.add(rotation_matrix @ position_matrix, addition_matrix)

        new_x = ((cos(omega*dt) * (x-ICCx)) + (-sin(omega*dt) * (y-ICCy))) + ICCx
        new_y = ((sin(omega*dt) * (x-ICCx)) + (cos(omega*dt) * (y-ICCy))) + ICCy
        new_theta = theta + (omega*dt)

    ev3.screen.clear()
    ev3.screen.draw_text(50, 20, "Time: {}".format(time_taken))
    ev3.screen.draw_text(50, 40, "x: {}".format(new_x))
    ev3.screen.draw_text(50, 60, "y: {}".format(new_y))
    ev3.screen.draw_text(50, 80, "theta: {}".format(new_theta))

    return new_x, new_y, new_theta


# Objective 1
start_time = time.time()
while not bump_sensor.pressed():
    left_motor.run(200)
    right_motor.run(200)
    
left_motor.stop()
right_motor.stop()

#ev3.screen.clear()
time_taken = time.time() - start_time
print(time.time() - start_time)
#ev3.screen.draw_text(50, 20, "Time: {}".format(time_taken))

new_x, new_y, new_theta = calculate_pose(200, 50, pi/2, time_taken, 200, 200, wheel_radius, L)

start_time = time.time()
rotations = 15 / wheel_circum
left_motor.run_angle(-200, rotations * 360, wait=False)
right_motor.run_angle(-200, rotations * 360)
time_taken = time.time() - start_time

new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, -200, -200, wheel_radius, L)

start_time = time.time()
left_motor.run_angle(250, 0.5 * 360, wait=False)
right_motor.run_angle(-250, 0.5 * 360)
#right_motor.run_angle(-250, 0.5 * 360)
time_taken = time.time() - start_time

hitpoint_x, hitpoint_y, hitpoint_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 250, -250, wheel_radius, L)

# Objective 2
adjustments = 0.3 * 360

start_time = time.time()
current_distance = (ultrasonic_sensor.distance()) / 10
if current_distance < 10:
    right_motor.run_angle(75, adjustments, wait=False)
    left_motor.run_angle(100, adjustments)
    right_motor.brake()
    time_taken = time.time() - start_time
    new_x, new_y, new_theta = calculate_pose(hitpoint_x, hitpoint_y, hitpoint_theta, time_taken, 100, 75, wheel_radius, L)
elif current_distance > 20:
    left_motor.run_angle(75, adjustments, wait=False)
    right_motor.run_angle(100, adjustments)
    left_motor.brake()
    time_taken = time.time() - start_time
    new_x, new_y, new_theta = calculate_pose(hitpoint_x, hitpoint_y, hitpoint_theta, time_taken, 75, 100, wheel_radius, L)
else:
    left_motor.run_angle(100, adjustments, wait=False)
    right_motor.run_angle(100, adjustments)
    time_taken = time.time() - start_time
    new_x, new_y, new_theta = calculate_pose(hitpoint_x, hitpoint_y, hitpoint_theta, time_taken, 100, 100, wheel_radius, L)

new_distance = (ultrasonic_sensor.distance()) / 10

flag = True
while flag:
#while True:
    #diff = abs(new_distance - current_distance) / 2
    buttonPress()

    start_time = time.time()
    if new_distance < current_distance:
        right_motor.run_angle(75, adjustments, wait=False)
        left_motor.run_angle(100, adjustments)
        right_motor.brake()
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 100, 75, wheel_radius, L)

    elif new_distance > current_distance:
        left_motor.run_angle(75, adjustments, wait=False)
        right_motor.run_angle(100, adjustments)
        left_motor.brake()
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 75, 100, wheel_radius, L)
    else:
        left_motor.run_angle(100, adjustments, wait=False)
        right_motor.run_angle(100, adjustments)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 100, 100, wheel_radius, L)

    start_time = time.time()
    current_distance = new_distance
    new_distance = (ultrasonic_sensor.distance()) / 10

    if new_distance < 10:
        left_motor.run_angle(360, 0.5 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(-360, 0.5 * 360)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 360, -360, wheel_radius, L)

        start_time = time.time()
        left_motor.run_angle(300, 0.5 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(300, 0.5 * 360)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 300, 300, wheel_radius, L)

        start_time = time.time()
        left_motor.run_angle(-360, 0.5 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(360, 0.5 * 360)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, -360, 360, wheel_radius, L)

    elif new_distance > 20:
        left_motor.run_angle(-360, 0.25 * 360, wait=False)
        buttonPress()
        right_motor.run_angle(360, 0.25 * 360)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, -360, 360, wheel_radius, L)

        if ((ultrasonic_sensor.distance()) / 10) >= new_distance:
            start_time = time.time()
            left_motor.run_angle(-360, 0.25 * 360, wait=False)
            buttonPress()
            right_motor.run_angle(360, 0.25 * 360)
            time_taken = time.time() - start_time
            new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, -360, 360, wheel_radius, L)

            start_time = time.time()
            left_motor.run_angle(300, 0.5 * 360, wait=False)
            buttonPress()
            right_motor.run_angle(300, 0.5 * 360)
            time_taken = time.time() - start_time
            new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 300, 300, wheel_radius, L)

            start_time = time.time()
            left_motor.run_angle(360, 0.5 * 360, wait=False)
            buttonPress()
            right_motor.run_angle(-360, 0.5 * 360)
            time_taken = time.time() - start_time
            new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 360, -360, wheel_radius, L)

    if abs(new_x - hitpoint_x) <= 5 and abs(new_y - hitpoint_y) <= 5:
        flag = False
    #sleep(0.2)

while new_theta != 0:
    if new_theta > 0:
        start_time = time.time()
        left_motor.run_angle(150, 1, wait=False)
        right_motor.run_angle(-150, 1)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, 150, -150, wheel_radius, L)
    else:
        start_time = time.time()
        left_motor.run_angle(-150, 1, wait=False)
        right_motor.run_angle(150, 1)
        time_taken = time.time() - start_time
        new_x, new_y, new_theta = calculate_pose(new_x, new_y, new_theta, time_taken, -150, 150, wheel_radius, L)

x_offset = 200 - new_x
y_offset = new_y - 50
rotations = x_offset / wheel_circum
left_motor.run_angle(200, rotations * 360, wait=False)
right_motor.run_angle(200, rotations * 360)

left_motor.run_angle(250, 180, wait=False)
right_motor.run_angle(-250, 180)
rotations = y_offset / wheel_circum
left_motor.run_angle(200, rotations * 360, wait=False)
right_motor.run_angle(200, rotations * 360)

ev3.speaker.beep()