import sys
sys.path.append('/home/pi/cs437-picar')

# import time
from typing import Dict, Callable
from picar_4wd.types import MotorPower, GrayscaleResult, GrayscaleReading
import picar_4wd as fc
import sys
import signal

car_speed: MotorPower = 2
greyscale_ref: GrayscaleReading = 150
DIST_ABOVE_REF = 2
min_dist = 10  # [cm]
sleep_duration = 0.1  # [s]


def detect_obstacles():
    scan_result = fc.scan_step(min_dist)
    if not scan_result:
        return False
    curr_scan = scan_result[3:7]
    if curr_scan != [DIST_ABOVE_REF, DIST_ABOVE_REF, DIST_ABOVE_REF, DIST_ABOVE_REF]:
        fc.time.sleep(sleep_duration)
        fc.backward(car_speed)
        fc.time.sleep(sleep_duration)
        fc.turn_left(car_speed)
        return True
    else:
        return False


action_dict: Dict[GrayscaleResult, Callable[[MotorPower], None]] = {
    GrayscaleResult.LEFT: fc.turn_left,
    GrayscaleResult.FORWARD: fc.forward,
    GrayscaleResult.RIGHT: fc.turn_right,
}


def track_line(is_detour: bool) -> None:
    adc_value_list = fc.get_grayscale_list()
    line_status = fc.get_line_status(greyscale_ref, adc_value_list)

    '''
    call detect_obstacles with the suggested direction from line_status 
    then in return it either continue with this direction or move to the left
    if move to the left then need again to verify it crossed the obstacle
    and then need to move to the right back to path
    '''
    # if is_detour:
    #     fc.turn_right(car_speed)
    #     # return False
    # else:
    if line_status is not GrayscaleResult.UNKNOWN:
        action_dict[line_status](car_speed)
        # time.sleep(0.05)
        # fc.stop()
    else:
        fc.stop()
    # if detect_obstacles():
    #     return True
    # return last_state


def signal_handler(signal, frame):
    fc.stop()
    print("\nprogram exiting gracefully")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    '''
    This is main script for cs437 lab1 part1
    Driving along a routh which determined by path marked on the floor
    Note that track_line is based on picar-4wd Track_line implementation
    While driving avoiding obstacles, 
    and if encountered one go left and then turn right and return to path
    '''

    while True:
        track_line(False)