import time

import picar_4wd as fc

turn_fact = 0.01
fw_fact = 0.04


if __name__ == "__main__":
    fc.turn_right(1)
    time.sleep(turn_fact * 45)
    fc.stop()

    fc.turn_left(1)
    time.sleep(turn_fact * 45)
    fc.stop()

    fc.forward(1)
    time.sleep(fw_fact * 20)
    fc.stop()
