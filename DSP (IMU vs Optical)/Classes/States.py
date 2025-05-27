from enum import Enum, auto

class CalibrationState(Enum):
    POSE_DEFAULT = auto()
    COUNTDOWN = auto()
    COMPLETED = auto()
    FAILED = auto()