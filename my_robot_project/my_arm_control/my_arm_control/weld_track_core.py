"""ROS-independent calculations for weld tracking; standard-library only."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from statistics import median
from typing import Iterable, Sequence


class PlannerState(str, Enum):
    IDLE = 'idle'
    RECORDING = 'recording'
    PLANNING = 'planning'
    EXECUTING = 'executing'
    ERROR = 'error'


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float

    def is_valid(self) -> bool:
        return all(isfinite(value) for value in (self.fx, self.fy, self.cx, self.cy)) and self.fx > 0 and self.fy > 0


def pixel_to_camera_point(u, v, depth_image, camera: CameraModel, *, depth_scale=0.001,
                          patch_radius=2, min_depth_m=0.05, max_depth_m=2.0):
    """Return an optical-frame (x, y, z) tuple in metres, or None if invalid.

    Both nested Python sequences and NumPy 2-D arrays are accepted.
    """
    if depth_image is None or not camera.is_valid() or depth_scale <= 0:
        return None
    try:
        height, width = depth_image.shape[:2] if hasattr(depth_image, 'shape') else (len(depth_image), len(depth_image[0]))
        x, y = int(round(u)), int(round(v))
        if height <= 0 or width <= 0 or not (0 <= x < width and 0 <= y < height):
            return None
        values = []
        for row_index in range(max(0, y - patch_radius), min(height, y + patch_radius + 1)):
            for column_index in range(max(0, x - patch_radius), min(width, x + patch_radius + 1)):
                value = float(depth_image[row_index][column_index]) * depth_scale
                if isfinite(value) and min_depth_m <= value <= max_depth_m:
                    values.append(value)
    except (IndexError, TypeError, ValueError):
        return None
    if not values:
        return None
    z = float(median(values))
    return ((float(u) - camera.cx) * z / camera.fx, (float(v) - camera.cy) * z / camera.fy, z)


def resample_trajectory(points: Sequence[Sequence[float]], count: int = 10):
    """Linearly resample finite 3-D points and preserve first/last endpoints."""
    if len(points) < 2 or count < 2:
        raise ValueError('trajectory must contain at least two 3D points')
    try:
        raw = [tuple(float(value) for value in point) for point in points]
    except (TypeError, ValueError) as error:
        raise ValueError('trajectory must contain numeric 3D points') from error
    if any(len(point) != 3 or not all(isfinite(value) for value in point) for point in raw):
        raise ValueError('trajectory must contain finite 3D points')
    result = []
    intervals = len(raw) - 1
    for target_index in range(count):
        position = target_index * intervals / (count - 1)
        left = min(int(position), intervals - 1)
        ratio = position - left if target_index < count - 1 else 1.0
        result.append(tuple(raw[left][axis] + ratio * (raw[left + 1][axis] - raw[left][axis]) for axis in range(3)))
    return result


def point_in_workspace(point: Iterable[float], minimum: Sequence[float], maximum: Sequence[float]) -> bool:
    try:
        point, minimum, maximum = tuple(map(float, point)), tuple(map(float, minimum)), tuple(map(float, maximum))
    except (TypeError, ValueError):
        return False
    return (len(point) == len(minimum) == len(maximum) == 3 and all(isfinite(value) for value in point)
            and all(low <= high for low, high in zip(minimum, maximum))
            and all(low <= value <= high for value, low, high in zip(point, minimum, maximum)))


def validate_pulses(pulses: Sequence[int], count=4, low=0, high=1000):
    try:
        values = tuple(int(value) for value in pulses[:count])
    except (TypeError, ValueError):
        return None
    return values if len(values) == count and all(low <= value <= high for value in values) else None
