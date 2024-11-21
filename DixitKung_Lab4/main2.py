#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile

from utime import sleep
import time
from math import pi, sin, cos, sqrt, atan

ev3 = EV3Brick()

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
ultrasonic_sensor = UltrasonicSensor(Port.S1)
bump_sensor1 = TouchSensor(Port.S4)
bump_sensor2 = TouchSensor(Port.S3)

wheel_d = 5.6
x_pos = 50
y_pos = 0
theta = 0


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

        new_x = ((cos(omega*dt) * (x-ICCx)) + (-sin(omega*dt) * (y-ICCy))) + ICCx
        new_y = ((sin(omega*dt) * (x-ICCx)) + (cos(omega*dt) * (y-ICCy))) + ICCy
        new_theta = theta + (omega*dt)

    ev3.screen.clear()
    ev3.screen.draw_text(50, 40, "x: {}".format(new_x))
    ev3.screen.draw_text(50, 60, "y: {}".format(new_y))
    ev3.screen.draw_text(50, 80, "theta: {}".format(new_theta))
    #print("x pos: " + str(new_x))
    #print("y pos: " + str(new_y))
    #print("theta: " + str(new_theta))  

    return new_x, new_y, new_theta



def stop():
    left_motor.stop()
    right_motor.stop()

def move_forward():
    global x_pos, y_pos, theta

    distance_to_goal = sqrt((250 - x_pos)**2 + (250 - y_pos)**2)
    theta = atan((250 - y_pos) / (250 - x_pos))

    left_motor.run_angle(200, -theta * 180 / pi * 2.3, wait=False)
    right_motor.run_angle(200, theta * 180 / pi * 2.3, wait=True)

    while not (bump_sensor1.pressed() or bump_sensor2.pressed()):
        start_time = time.time()
        left_motor.run_time(500, 500, wait=False)
        right_motor.run_time(500, 500, wait=True)
        time_taken = time.time() - start_time

        x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, time_taken, 500, 500, wheel_d / 2, 12)

        if (x_pos >= 420 and x_pos <= 480) and (y_pos >= 420 and y_pos <= 480):
            stop()
            return

        wait(50)
    stop()
    ev3.speaker.beep()
    wait(1000)

def reverse():
    global x_pos, y_pos, theta
    left_motor.run_time(-300, 750, Stop.BRAKE, wait=False)
    right_motor.run_time(-300, 750, Stop.BRAKE, wait=True)
    time_taken = 650 / 1000
    x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, time_taken, -300, -300, wheel_d / 2, 12)

    left_motor.run_angle(300, 260, Stop.BRAKE, wait=False)
    right_motor.run_angle(-300, 200, Stop.BRAKE, wait=True)
    time_taken = (260 / 300)
    x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, time_taken, 300, -300, wheel_d / 2, 12)

def align_to_target():
    global theta
    target_theta = atan(250 / 200)
    if theta != 0:
        left_motor.run_angle(200, 145 * 2.3, wait=False)
        right_motor.run_angle(200, -145 * 2.3 , wait=True)
    #left_motor.run_angle(200, -target_theta * 180 / pi * 2.3, wait=False)
    #right_motor.run_angle(200, target_theta * 180 / pi * 2.3, wait=True)
    theta = target_theta

def wall_following():
    global x_pos, y_pos, theta

    num = 0
    while True:
        num += 1
        if (ultrasonic_sensor.distance() / 10) < 30:
            left_motor.run_time(160, 1000, wait=False)
            buttonPress()
            right_motor.run_time(105, 1000)
            x_pos, x_pos, theta = calculate_pose(x_pos, x_pos, theta, 1 - 0.3, 160, 105, wheel_d / 2, 12)
        else:
            left_motor.run_time(105, 1000, wait=False)
            buttonPress()
            right_motor.run_time(160, 1000)
            x_pos, x_pos, theta = calculate_pose(x_pos, x_pos, theta, 1 - 0.3, 105, 160, wheel_d / 2, 12)

        
        target_slope = 250 / 200
        current_slope = (y_pos - 0) / (x_pos - 50) if (x_pos - 50) != 0 else float('inf')
        print(num)
        print(abs(current_slope - target_slope))
        if num >= 25 and abs(current_slope - target_slope) < 0.35:
            num = 0
            print("Reached the target line. Aligning to target direction.")
            stop()
            align_to_target()
            return

        wait(50)

    stop()
    ev3.speaker.beep()

def buttonPress():
    if bump_sensor1.pressed() or bump_sensor2.pressed():
        reverse()




while (x_pos < 420 or x_pos > 480) or (y_pos < 420 or y_pos > 480):
    move_forward()
    if (x_pos >= 420 and x_pos <= 480) and (y_pos >= 420 and y_pos <= 480):
        break
    reverse()
    wall_following()

wait(1000000)
ev3.speaker.beep()