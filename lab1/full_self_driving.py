import os.path
import sys
sys.path.append('/home/pi/cs437-picar')
import time
import datetime

import cv2

import picar_4wd as fc
from mapping import Mapper, MapCoordinate, AngleDeg
from road_sign_detector import Detector, RoadSign

TURN_FACT = 0.01  # sec / deg
FW_FACT = 0.04  # sec / cm
UPDATE_FREQUENCY = 4
im_res = (320, 240)
save_movie = True
sign_detector = Detector()


def stop_on_sign() -> None:
    _, frame = stream.read()
    bb = sign_detector.detected(frame, RoadSign.Stop)
    while len(bb[RoadSign.Stop]) > 0:
        time.sleep(0.5)
        if save_movie:
            for (x, y, w, h) in bb[RoadSign.Stop]:
                cv2.rectangle(frame, (x, y), (x + w, y + h), RoadSign.Stop.value, 2)
                cv2.putText(frame, "STOP", (x, y), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1.0,
                            color=RoadSign.Stop.value)
            out.write(frame)
        _, frame = stream.read()
        bb = sign_detector.detected(frame, RoadSign.Stop)
        print("Stop")
    if save_movie:
        out.write(frame)


if __name__ == '__main__':
    output_path = f"./run {datetime.datetime.now().strftime('%H:%M:%S')}"
    os.makedirs(output_path, exist_ok=True)

    # initiate mapper
    mapper = Mapper(output_path=output_path)
    car_position = MapCoordinate(0, 0)
    car_heading: AngleDeg = 0
    destination_a = MapCoordinate(50, 80)
    destination_b = MapCoordinate(-60, 120)
    # initiate detector
    stream = cv2.VideoCapture(0)
    stream.set(3, im_res[0])
    stream.set(4, im_res[1])

    if save_movie:
        out = cv2.VideoWriter(
            os.path.join(output_path, 'movie.avi'),
            cv2.VideoWriter_fourcc(*'XVID'),
            20,
            im_res
        )
    (grabbed, frame) = stream.read()
    if grabbed:
        print("Camera OK")
        print(f"Image resolution: {frame.shape[:2]} pixels")
    else:
        raise "problem with camera"

    counter = 0
    for destination in [destination_a, destination_b]:
        mapper.update_grid(car_position, car_heading)
        maneuvers = mapper.plot_navigation_plan(destination)
        new_destination = True

        while len(maneuvers) > 0:
            stop_on_sign()

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
            stop_on_sign()

            fc.forward(1)
            time.sleep(FW_FACT * maneuver.forward)
            fc.stop()
            stop_on_sign()

            # assuming all is well this is the car position
            car_position = maneuver.end_position

            counter += 1
            if (counter >= UPDATE_FREQUENCY or new_destination) and len(maneuvers) > 0:
                counter = 0
                new_destination = False
                mapper.update_grid(car_position, car_heading)
                maneuvers = mapper.plot_navigation_plan(destination)

