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


ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
ultrasonic_sensor = UltrasonicSensor(Port.S1)
bump_sensor = TouchSensor(Port.S4)

# pid params
Kp = 2.0
Ki = 0.0
Kd = 0.5

target_distance = 560
wheel_d = 5.6
wheel_circ = wheel_d * 3.1416



def calculate_distance_from_degrees(degrees):
    return (degrees / 360) * wheel_circ

distance_to_travel = 200

def drive(speed, steering):
    max_speed = 300
    left_speed = max(min(speed + steering, max_speed), -max_speed)
    right_speed = max(min(speed - steering, max_speed), -max_speed)
    left_motor.run(left_speed)
    right_motor.run(right_speed)


def stop():
    left_motor.stop()
    right_motor.stop()


def move_forward_until_bump():
    ev3.speaker.beep()
    while not bump_sensor.pressed():
        drive(300, 0)
    stop()
    ev3.speaker.beep()
    wait(1000)


def back_up_and_turn_right():
    left_motor.run_time(-300, 750, Stop.BRAKE, wait=True)
    right_motor.run_time(-300, 750, Stop.BRAKE, wait=True)
    left_motor.run_angle(300, 260, Stop.BRAKE, wait=True)
    right_motor.run_angle(-300, 260, Stop.BRAKE, wait=True)


def wall_following():
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    last_error = 0
    last_time = time.time()
    integral = 0

    while True:
        left_degrees = left_motor.angle()
        right_degrees = right_motor.angle()
        average_degrees = (abs(left_degrees) + abs(right_degrees)) / 2
        total_distance_traveled = calculate_distance_from_degrees(average_degrees)

        if total_distance_traveled >= distance_to_travel:
            print("Desired distance traveled. Stopping.")
            break

        distance_mm = ultrasonic_sensor.distance()

        if distance_mm <= 0 or distance_mm > 2550:
            stop()
            print("Invalid sensor reading. Stopping the robot.")
            break

        distance_cm = distance_mm / 10
        error = target_distance - distance_cm
        error_scaled = error * 10
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time
        integral += error_scaled * delta_time
        derivative = (error_scaled - last_error) / delta_time if delta_time > 0 else 0
        last_error = error_scaled
        steering = (Kp * error_scaled) + (Ki * integral) + (Kd * derivative)
        max_steering = 200
        steering = max(min(steering, max_steering), -max_steering)
        speed = 200


        if bump_sensor.pressed():
            print("Bump detected during wall following. Reversing and turning right.")
            stop()
            back_up_and_turn_right()
            continue

        drive(speed, steering)
        wait(50)

    stop()
    ev3.speaker.beep()


ev3.speaker.beep()

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)



ev3.speaker.beep()
move_forward_until_bump()
back_up_and_turn_right()
wall_following()