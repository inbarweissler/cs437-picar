from enum import Enum

GrayscaleReading = int
MotorPower = int


class GrayscaleResult(Enum):
    LEFT = 0
    FORWARD = 1
    RIGHT = 2
    UNKNOWN = -1
