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

# pid params
Kp = 2.0
Ki = 0.0
Kd = 0.5

target_distance = 14
wheel_d = 5.6
wheel_circ = wheel_d * 3.1416

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
    ev3.screen.draw_text(50, 20, "Time: {}".format(dt))
    ev3.screen.draw_text(50, 40, "x: {}".format(new_x))
    ev3.screen.draw_text(50, 60, "y: {}".format(new_y))
    ev3.screen.draw_text(50, 80, "theta: {}".format(new_theta))

    return new_x, new_y, new_theta

def calculate_distance_from_degrees(degrees):
    return (degrees / 360) * wheel_circ

distance_to_travel = 260

def drive(speed, steering):
    max_speed = 300
    left_speed = max(min(speed + steering, max_speed), -max_speed)
    right_speed = max(min(speed - steering, max_speed), -max_speed)
    left_motor.run(left_speed)
    right_motor.run(right_speed)

    return left_speed, right_speed


def stop():
    left_motor.stop()
    right_motor.stop()


def move_forward():
    global x_pos, y_pos, theta

    if theta > 0:
        left_motor.run_angle(200, theta * 180 / pi * 2.3, wait=False)
        right_motor.run_angle(200, -theta * 180 / pi * 2.3, wait=True)
    elif theta < 0:
        left_motor.run_angle(200, -theta * 180 / pi * 2.3, wait=False)
        right_motor.run_angle(200, theta * 180 / pi * 2.3, wait=True)

    distance_to_goal = sqrt((250 - x_pos)**2 + (250 - y_pos)**2)
    theta = atan((250 - y_pos) / (250 - x_pos))

    # Adjust the robot's heading angle
    left_motor.run_angle(200, -theta * 180 / pi * 2.3, wait=False)
    right_motor.run_angle(200, theta * 180 / pi * 2.3, wait=True)

    while not (bump_sensor1.pressed() or bump_sensor2.pressed()):
        #rotations = distance_to_goal / wheel_circ
        #rotation_angle = rotations * 360
        start_time = time.time()
        left_motor.run_time(500, 500, wait=False)
        right_motor.run_time(500, 500, wait=True)

        #left_motor.stop()
        #right_motor.stop()
        time_taken = time.time() - start_time

        # Update position
        x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, 0.5 - 0.3, 500, 500, wheel_d / 2, 12)

        # Check if the robot is within the goal region
        if (x_pos < 260 and x_pos > 240) and (y_pos < 260 and y_pos > 240):
            wait(10000000)
            break

        wait(50)
        # wait(10000000)
    stop()
    ev3.speaker.beep()
    wait(1000)


def back_up_and_turn_right():
    global x_pos, y_pos, theta
    left_motor.run_time(-300, 750, Stop.BRAKE, wait=True)
    right_motor.run_time(-300, 750, Stop.BRAKE, wait=True)
    time_taken = 750 / 1000
    x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, time_taken, -300, -300, wheel_d / 2, 12)

    left_motor.run_angle(300, 260, Stop.BRAKE, wait=True)
    right_motor.run_angle(-300, 260, Stop.BRAKE, wait=True)
    time_taken = (260 / 300)
    x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, time_taken, 300, -300, wheel_d / 2, 12)


## version 1
def wall_following():
    global x_pos, y_pos, theta

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    last_error = 0
    last_time = time.time()
    integral = 0

    while True:
        left_degrees = left_motor.angle()
        right_degrees = right_motor.angle()
        average_degrees = (abs(left_degrees) + abs(right_degrees)) / 2
        #total_distance_traveled = calculate_distance_from_degrees(average_degrees)

        #if total_distance_traveled >= distance_to_travel:
        #    print("Desired distance traveled. Stopping.")
        #    break

        distance_mm = ultrasonic_sensor.distance()

        if distance_mm <= 0 or distance_mm > 2550:
            stop()
            print("Invalid sensor reading. Stopping the robot.")
            break
        elif distance_mm > 1000:
            print(distance_mm)
            #wait(1000000000)
            # x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, delta_time, 500, 500, wheel_d/2, 12)
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
        speed = 150


        if bump_sensor1.pressed() or bump_sensor2.pressed():
            print("Bump detected during wall following. Reversing and turning right.")
            stop()
            back_up_and_turn_right()
            continue


        var1, var2 = drive(speed, steering)
        x_pos, y_pos, theta = calculate_pose(x_pos, y_pos, theta, delta_time, var1, var2, wheel_d/2, 12)
        wait(50)

    stop()
    ev3.speaker.beep()



## version 2
# def wall_following():
#     left_motor.reset_angle(0)
#     right_motor.reset_angle(0)

#     last_error = 0
#     last_steering = 0
#     integral = 0
#     last_time = time.time()

#     while True:
#         left_degrees = left_motor.angle()
#         right_degrees = right_motor.angle()
#         average_degrees = (abs(left_degrees) + abs(right_degrees)) / 2
#         total_distance_traveled = calculate_distance_from_degrees(average_degrees)

#         if total_distance_traveled >= distance_to_travel:
#             print("Desired distance traveled. Stopping.")
#             break

#         distance_mm = ultrasonic_sensor.distance()
#         if distance_mm <= 0 or distance_mm > 2550:
#             print("Invalid sensor reading. Ignoring current reading.")
#             continue

#         distance_cm = distance_mm / 10
#         error = target_distance - distance_cm
#         error_scaled = error * 10
#         current_time = time.time()
#         delta_time = current_time - last_time
#         last_time = current_time
#         integral += error_scaled * delta_time
#         derivative = (error_scaled - last_error) / delta_time if delta_time > 0 else 0
#         last_error = error_scaled

#         steering = (Kp * error_scaled) + (Ki * integral) + (Kd * derivative)
#         steering = 0.7 * last_steering + 0.3 * steering  # Smooth adjustment
#         last_steering = steering

#         max_steering = 100
#         steering = max(min(steering, max_steering), -max_steering)
#         speed = 150  # Reduced speed for better control

#         if bump_sensor1.pressed() or bump_sensor2.pressed():
#             print("Bump detected during wall following. Reversing and turning right.")
#             stop()
#             back_up_and_turn_right()
#             continue

#         drive(speed, steering)
#         wait(50)

#     stop()
#     ev3.speaker.beep()


# ev3.speaker.beep()

# while Button.CENTER not in ev3.buttons.pressed():
#     wait(10)


# ev3.speaker.beep()
# #move_forward_until_bump()
# #back_up_and_turn_right()
# #wall_following()

# distance_to_goal = 320.156
# theta = 0.896 # arctan(2.5/2)

# left_motor.run_angle(200, -theta * 180/pi * 2.3, wait=False)
# right_motor.run_angle(200, theta * 180/pi * 2.3)

# start_time = time.time()
# rotations = distance_to_goal / wheel_circ
# left_motor.run_angle(500, rotations * 360, wait=False)
# right_motor.run_angle(500, rotations * 360)
# left_motor.stop()
# right_motor.stop()
# time_taken = time.time() - start_time

# new_x, new_y, new_theta = calculate_pose(50, 0, theta, time_taken, 500, 500, wheel_d/2, 12)

# wait(10000000)








while (x_pos > 260 or x_pos < 240) and (y_pos > 260 or y_pos < 240):
    move_forward()
    if (x_pos < 260 and x_pos > 240) and (y_pos < 260 and y_pos > 240):
        break
    back_up_and_turn_right()
    wall_following()
    
wait(1000000)