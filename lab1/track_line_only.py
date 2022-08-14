import sys
sys.path.append('/home/pi/cs437-picar')
import time
from typing import Dict, Callable
from picar_4wd.types import MotorPower, GrayscaleResult, GrayscaleReading, DistanceStatus
import picar_4wd as fc
import sys
import signal

car_speed: MotorPower = 1
sleep_duration = 0.1  # [s]
max_dist = 20  # [cm]
scan_res = 15  # [deg]
wide_field_of_view = 60  # [deg]
narrow_field_of_view = 30  # [deg]
debug_flag = True


class LineTracer:
    counter_unknown = 0
    greyscale_ref: GrayscaleReading = 150
    action_dict: Dict[GrayscaleResult, Callable[[MotorPower], None]] = {
        GrayscaleResult.LEFT: fc.turn_left,
        GrayscaleResult.FORWARD: fc.forward,
        GrayscaleResult.RIGHT: fc.turn_right,
    }

    def __init__(self, car_speed, sleep_duration):
        self.car_speed = car_speed
        self.sleep_duration = sleep_duration
        pass

    def trace(self) -> None:
        adc_value_list = fc.get_grayscale_list()
        line_status = fc.get_line_status(self.greyscale_ref, adc_value_list)

        if line_status is not GrayscaleResult.UNKNOWN:
            self.reset_counter()
            self.action_dict[line_status](self.car_speed)
        else:
            fc.stop()
            self.update_counter()
            if self.counter_unknown > 10:
                for _ in range(10):
                    fc.turn_left(self.car_speed)
                    time.sleep(self.sleep_duration)
                    fc.stop()
                    line_status = fc.get_line_status(self.greyscale_ref, adc_value_list)
                    if line_status is not GrayscaleResult.UNKNOWN:
                        return

    def update_counter(self):
        self.counter_unknown += 1

    def reset_counter(self):
        self.counter_unknown = 0


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
        # fc.servo.scan_step(step_res=scan_res, angle_range=narrow_field_of_view)
        status = fc.get_us_status(ref1=max_dist)
        # if debug_flag:
        #     fc.stop()

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
    fc.servo.set_angle_and_wait(0)
    line_tracer = LineTracer(car_speed, sleep_duration)
    not_blocked = True
    while not_blocked:
        line_tracer.trace()
        if detect_obstacles(False):
            fc.stop()
            not_blocked = False
    print('No way to go')
