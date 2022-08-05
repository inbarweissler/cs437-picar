import picar_4wd as fc
from enum import Enum

car_speed = 20
greyscale_ref = 200
DIST_ABOVE_REF = 2
min_dist = 20  # [cm]
sleep_duration = 0.1  # [s]



class GreyscaleRes(Enum):
    LEFT: int = -1
    FORWARD: int = 0
    RIGHT: int = 1


def detect_obstacles():
    scan_result = fc.scan_step(min_dist)
    if not scan_result:
        return False
    curr_scan = scan_result[3:7]
    if curr_scan != all(elem == DIST_ABOVE_REF for elem in curr_scan):
        fc.time.sleep(sleep_duration)
        fc.backward(car_speed)
        fc.time.sleep(sleep_duration)
        fc.turn_left(car_speed)
        return True
    else:
        fc.forward(car_speed)
        return False


def track_line(is_detour: bool) -> bool:
    adc_value_list = fc.get_grayscale_list()
    line_status = fc.get_line_status(greyscale_ref, adc_value_list)

    '''
    call detect_obstacles with the suggested direction from line_status 
    then in return it either continue with this direction or move to the left
    if move to the left then need again to verify it crossed the obstacle
    and then need to move to the right back to path
    '''
    if is_detour:
        fc.turn_right(car_speed)
        is_detour = False
    else:
        if line_status == GreyscaleRes.FORWARD:
            fc.forward(car_speed)
        elif line_status == GreyscaleRes.LEFT:
            fc.turn_left(car_speed)
        elif line_status == GreyscaleRes.RIGHT:
            fc.turn_right(car_speed)
        else:
            fc.stop()
    if detect_obstacles():
        is_detour = True


if __name__=='__main__':
    '''
    This is main script for cs437 lab1 part1
    Driving along a routh which determined by path marked on the floor
    Note that track_line is based on picar-4wd Track_line implementation
    While driving avoiding obstacles, 
    and if encountered one go left and then turn right and return to path
    '''
    is_detour = False
    while True:
        is_detour = track_line(is_detour)


