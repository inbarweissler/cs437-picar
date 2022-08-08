#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing import Tuple

# import numpy as np
from numpy import argmin, mean

from picar_4wd.pwm import PWM
from picar_4wd.adc import ADC
from picar_4wd.pin import Pin
from picar_4wd.motor import Motor
from picar_4wd.servo import Servo
from picar_4wd.types import GrayscaleReading, GrayscaleResult, DistanceStatus
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.speed import Speed
from picar_4wd.filedb import FileDB
from picar_4wd.utils import *


# Config File:
config = FileDB("config")
left_front_reverse = config.get('left_front_reverse', default_value=False)
right_front_reverse = config.get('right_front_reverse', default_value=False)
left_rear_reverse = config.get('left_rear_reverse', default_value=False)
right_rear_reverse = config.get('right_rear_reverse', default_value=False)
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value=0))

# Init motors
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=left_front_reverse)  # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=right_front_reverse)  # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=left_rear_reverse)  # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=right_rear_reverse)  # motor 4

# left_front_speed = Speed(12)
# right_front_speed = Speed(16)
left_rear_speed = Speed(25)
right_rear_speed = Speed(4)

# Init Greyscale
gs0 = ADC('A5')
gs1 = ADC('A6')
gs2 = ADC('A7')

# Init Ultrasonic
us = Ultrasonic(Pin('D8'), Pin('D9'))

# Init Servo
# print("Init Servo: %s" % ultrasonic_servo_offset)

servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)


def start_speed_thread():
    # left_front_speed.start()
    # right_front_speed.start()
    left_rear_speed.start()
    right_rear_speed.start()


##################################################################
# Grayscale
def get_grayscale_list() -> Tuple[GrayscaleReading, GrayscaleReading, GrayscaleReading]:
    return gs0.read(), gs1.read(), gs2.read()


def is_on_edge(ref: GrayscaleReading, gs_list):
    ref = int(ref)
    if gs_list[2] <= ref or gs_list[1] <= ref or gs_list[0] <= ref:
        return True
    else:
        return False


def get_line_status(ref: GrayscaleReading, fl_list) -> GrayscaleResult:  # 170<x<300
    ref = int(ref)
    line_detected = tuple(fl_list[i] < ref for i in range(len(fl_list)))
    if any(line_detected):
        if not line_detected[0]:
            return GrayscaleResult.RIGHT
        elif not line_detected[2]:
            return GrayscaleResult.LEFT
        else:
            return GrayscaleResult.FORWARD
    else:
        min_ind = argmin(fl_list)
        if fl_list[min_ind] / mean(fl_list) < 0.8:
            if min_ind == 0:
                return GrayscaleResult.LEFT
            elif min_ind == 2:
                return GrayscaleResult.RIGHT
    return GrayscaleResult.UNKNOWN


########################################################
errors = []


def run_command(cmd=""):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    # print(result)
    # print(status)
    return status, result


def do(msg="", cmd=""):
    print(" - %s..." % (msg), end='\r')
    print(" - %s... " % (msg), end='')
    status, result = eval(cmd)
    # print(status, result)
    if status == 0 or status == None or result == "":
        print('Done')
    else:
        print('Error')
        errors.append("%s error:\n  Status:%s\n  Error:%s" %
                      (msg, status, result))


def get_us_status(ref1=35, ref2=10, num_scans=5) -> DistanceStatus:
    dist_list = [us.get_distance() for _ in range(num_scans)]
    if all(dist is None for dist in dist_list):
        return DistanceStatus.ABOVE_MAX
    dist = mean([dist for dist in dist_list if dist is not None])
    if dist:
        if dist > ref1:
            return DistanceStatus.ABOVE_MAX
        elif dist > ref2:
            return DistanceStatus.ABOVE_MIN
        else:
            return DistanceStatus.BELOW_MIN


########################################################
# Motors
def forward(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(power)
    right_rear.set_power(power)


def backward(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(-power)
    right_rear.set_power(-power)


def turn_left(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(power)
    right_rear.set_power(power)


def turn_right(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(-power)
    right_rear.set_power(-power)


def stop():
    left_front.set_power(0)
    left_rear.set_power(0)
    right_front.set_power(0)
    right_rear.set_power(0)


def set_motor_power(motor, power):
    if motor == 1:
        left_front.set_power(power)
    elif motor == 2:
        right_front.set_power(power)
    elif motor == 3:
        left_rear.set_power(power)
    elif motor == 4:
        right_rear.set_power(power)


# def speed_val(*arg):
#     if len(arg) == 0:
#         return (left_front_speed() + left_rear_speed() + right_front_speed() + right_rear_speed()) / 4
#     elif arg[0] == 1:
#         return left_front_speed()
#     elif arg[0] == 2:
#         return right_front_speed()
#     elif arg[0] == 3:
#         return left_rear_speed()
#     elif arg[0] == 4:
#         return right_rear_speed()

def speed_val():
    return (left_rear_speed() + right_rear_speed()) / 2.0


########################################################
if __name__ == '__main__':
    start_speed_thread()
    while 1:
        forward(1)
        time.sleep(0.1)
        print(speed_val())
