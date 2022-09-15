from enum import Enum
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2
import os


class RoadSign(Enum):
    Stop = (0, 0, 255)
    TurnRight = (0, 255, 0)
    TurnLeft = (255, 0, 0)


class Detector:
    _weights_dir = "/home/pi/cs437-picar/road_sign_detector/trained_weights"

    def __init__(self):
        self._detectors: Dict[RoadSign, cv2.CascadeClassifier] = {}
        weights_file = {
            RoadSign.Stop: "stopsign_classifier.xml",
            RoadSign.TurnRight: "turnRight_ahead.xml",
            RoadSign.TurnLeft: "turnLeft_ahead.xml"
        }
        for road_sign in RoadSign:
            file_name = os.path.join(self._weights_dir, weights_file[road_sign])
            assert os.path.exists(file_name), "classifier weights file not found"
            self._detectors[road_sign] = cv2.CascadeClassifier(file_name)

    def detected(self, image: np.ndarray, road_sign: Optional[RoadSign] = None) -> Dict[
        RoadSign, List[Tuple[int, int, int, int]]]:
        im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        bounding_boxes: Dict[RoadSign, List[Tuple[int, int, int, int]]] = {}
        if road_sign is not None:
            bounding_boxes[road_sign] = self._detectors[road_sign].detectMultiScale(im_gray, 1.02, 10)
        else:
            for road_sign in RoadSign:
                bounding_boxes[road_sign] = self._detectors[road_sign].detectMultiScale(im_gray, 1.02, 10)
        return bounding_boxes
