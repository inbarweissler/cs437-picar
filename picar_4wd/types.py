from enum import Enum

GrayscaleReading = int
MotorPower = int


class GrayscaleResult(Enum):
    LEFT = 0
    FORWARD = 1
    RIGHT = 2
    UNKNOWN = -1


class DirectionType(Enum):
    LEFT = 0
    RIGHT = 1


class DistanceStatus(Enum):
    BELOW_MIN = 0
    ABOVE_MIN = 1
    ABOVE_MAX = 2
