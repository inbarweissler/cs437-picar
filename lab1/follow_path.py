import sys
sys.path.append('/home/pi/cs437-picar')

# import time
from typing import Dict, Callable
from picar_4wd.types import MotorPower, GrayscaleResult, GrayscaleReading, DistanceStatus
import picar_4wd as fc
import sys
import signal

car_speed: MotorPower = 2
greyscale_ref: GrayscaleReading = 150

max_sleep_duration = 2  # [s]
fw_sleep_duration = 0.5
bw_sleep_duration = 0.25
# debug_flag = False
max_dist = 10  # [cm]
scan_res = 15  # [deg]
wide_field_of_view = 90  # [deg]
narrow_field_of_view = 45  # [deg]

def detect_obstacles(full_scan=False) -> bool:
    if full_scan:
        fc.servo.set_angle_and_wait(0)
        n_scans = wide_field_of_view // scan_res
        scans = []
        for _ in range(n_scans):
            fc.servo.scan_step(step_res=scan_res, angle_range=wide_field_of_view)
            scans.append(fc.get_us_status(ref1=max_dist) != DistanceStatus.ABOVE_MAX)
        return any(scans)
    else:
        fc.servo.scan_step(step_res=scan_res, angle_range=narrow_field_of_view)
        status = fc.get_us_status(ref1=max_dist)
        if status == DistanceStatus.ABOVE_MAX:
            return False
        else:
            return True


action_dict: Dict[GrayscaleResult, Callable[[MotorPower], None]] = {
    GrayscaleResult.LEFT: fc.turn_left,
    GrayscaleResult.FORWARD: fc.forward,
    GrayscaleResult.RIGHT: fc.turn_right,
}


def track_line() -> None:
    adc_value_list = fc.get_grayscale_list()
    line_status = fc.get_line_status(greyscale_ref, adc_value_list)

    if line_status is not GrayscaleResult.UNKNOWN:
        action_dict[line_status](car_speed)
    else:
        fc.stop()


def signal_handler(signal, frame):
    fc.stop()
    print("\nprogram exiting gracefully")
    sys.exit(0)


def detour_obstacle():
    print("\nobstacle detected")
    fc.stop()
    fc.backward(car_speed)
    fc.time.sleep(bw_sleep_duration)
    fc.stop()
    fc.turn_left(car_speed)
    fc.time.sleep(fw_sleep_duration)
    fc.stop()
    fc.forward(car_speed)
    fc.time.sleep(fw_sleep_duration)
    fc.stop()
    fc.turn_right(car_speed)
    fc.turn_right(car_speed)
    fc.time.sleep(fw_sleep_duration)
    fc.stop()
    fc.forward(car_speed)
    fc.time.sleep(fw_sleep_duration)
    fc.stop()


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    '''
    This is main script for cs437 lab1 part1
    Driving along a routh which determined by path marked on the floor
    While driving avoiding obstacles, 
    and if encountered one go left and then turn right and return to path
    '''
    full_scan = True
    while True:
        track_line()
        fc.time.sleep(fw_sleep_duration)
        fc.stop()
        # if detect_obstacles(full_scan):
        #     full_scan = True
        #     detour_obstacle()
        # else:
        #     full_scan = False
