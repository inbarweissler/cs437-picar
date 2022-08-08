import sys

sys.path.append('/home/pi/cs437-picar')

# import time
from typing import Dict, Callable
from picar_4wd.types import MotorPower, GrayscaleResult, GrayscaleReading, DirectionType, DistanceStatus
import picar_4wd as fc
import sys
import signal
import random

car_speed: MotorPower = 1
greyscale_ref: GrayscaleReading = 150

max_sleep_duration = 2  # [s]
fw_sleep_duration = 0.25
bw_sleep_duration = 0.125
max_dist = 35  # [cm]
scan_res = 15  # [deg]
wide_field_of_view = 90  # [deg]
narrow_field_of_view = 90  # [deg]

action_dict: Dict[DirectionType, Callable[[MotorPower], None]] = {
    DirectionType.LEFT: fc.turn_left,
    DirectionType.RIGHT: fc.turn_right,
}

direction_types = (DirectionType.LEFT, DirectionType.RIGHT)


def random_turn() -> None:
    # choose random direction
    next_direction = random.choice(direction_types)
    action_dict[next_direction](car_speed)
    # choose random time duration (define the angle)
    sleep_duration = random.random() * max_sleep_duration
    fc.time.sleep(sleep_duration)
    fc.stop()


def detect_obstacles(full_scan=False) -> bool:
    if full_scan:
        fc.servo.set_angle_and_wait(0)
        n_scans = wide_field_of_view // scan_res
        scans = []
        for _ in range(n_scans):
            fc.servo.scan_step(step_res=scan_res, angle_range=wide_field_of_view)
            scans.append(fc.get_us_status(ref1=max_dist, num_scans=10) != DistanceStatus.ABOVE_MAX)
        return any(scans)
    else:
        fc.servo.scan_step(step_res=scan_res, angle_range=narrow_field_of_view)
        status = fc.get_us_status(ref1=max_dist, num_scans=10)
        if status == DistanceStatus.ABOVE_MAX:
            return False
        else:
            return True


def signal_handler(signal, frame):
    fc.stop()
    print("\nprogram exiting gracefully")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    '''
    This is part of the result needed for cs437 lab1 part1
    Driving in a random direction, and scanning for obstacles
    When approaching an obstacle, move backward and choose new direction 
    '''
    full_scan = True
    while True:
        if not detect_obstacles(full_scan):
            full_scan = False
            fc.forward(car_speed)
            fc.time.sleep(fw_sleep_duration)
            fc.stop()
        else:
            fc.stop()
            # if not full_scan:
            #     fc.backward(car_speed)
            #     fc.time.sleep(bw_sleep_duration)
            #     fc.stop()
            random_turn()
            full_scan = True
