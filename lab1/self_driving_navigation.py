import os
import sys
from datetime import datetime

sys.path.append('/home/pi/cs437-picar')
import time
from mapping import Mapper, MapCoordinate
import picar_4wd as fc

TURN_FACT = 0.01  # sec / deg
FW_FACT = 0.04  # sec / cm
UPDATE_FREQUENCY = 4

if __name__ == '__main__':
    output_path = f"./run {datetime.now().strftime('%H:%M:%S')}"
    os.makedirs(output_path, exist_ok=True)

    mapper = Mapper(output_path=output_path)
    car_position = MapCoordinate(0, 0)
    destination_a = MapCoordinate(50, 80)
    destination_b = MapCoordinate(-60, 120)

    counter = 0
    car_heading = 0
    for destination in [destination_a, destination_b]:
        mapper.update_grid(car_position, car_heading)
        maneuvers = mapper.plot_navigation_plan(destination)
        new_destination = True

        while len(maneuvers) > 0:
            maneuver = maneuvers.pop(0)

            if maneuver.heading != car_heading:
                delta_ang = maneuver.heading - car_heading
                if delta_ang > 0:
                    fc.turn_right(1)
                else:
                    fc.turn_left(1)
                time.sleep(TURN_FACT * abs(delta_ang))
                fc.stop()
                car_heading = maneuver.heading

            fc.forward(1)
            time.sleep(FW_FACT * maneuver.forward)
            fc.stop()

            # assuming all is well this is the car position
            car_position = maneuver.end_position

            counter += 1
            if (counter >= UPDATE_FREQUENCY or new_destination) and len(maneuvers) > 0:
                counter = 0
                new_destination = False
                mapper.update_grid(car_position, car_heading)
                maneuvers = mapper.plot_navigation_plan(destination)

