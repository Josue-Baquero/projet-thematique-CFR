from dataclasses import dataclass
from typing import Tuple
from scipy.spatial.transform import Rotation as R

@dataclass
class RobotPose:
    position: Tuple[float, float, float]
    orientation: R
    timestamp: float