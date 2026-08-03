"""Pure-Python tests for safety-critical weld-track calculations."""

from my_arm_control.weld_track_core import (
    CameraModel,
    pixel_to_camera_point,
    point_in_workspace,
    resample_trajectory,
    validate_pulses,
)


def test_pixel_to_camera_point_uses_valid_median_depth():
    depth_image = [[1000 for _ in range(9)] for _ in range(9)]
    depth_image[4][4] = 0
    point = pixel_to_camera_point(4, 4, depth_image, CameraModel(100, 100, 4, 4))
    assert point == (0.0, 0.0, 1.0)


def test_resample_trajectory_preserves_endpoints():
    result = resample_trajectory([(0, 0, 0), (1, 2, 3)], count=5)
    assert len(result) == 5
    assert result[0] == (0.0, 0.0, 0.0)
    assert result[-1] == (1.0, 2.0, 3.0)


def test_workspace_and_servo_limits_reject_unsafe_values():
    assert point_in_workspace((0.3, -0.3, 0.05), (-0.3, -0.3, 0.05), (0.3, 0.3, 0.5))
    assert not point_in_workspace((0.31, 0, 0.2), (-0.3, -0.3, 0.05), (0.3, 0.3, 0.5))
    assert validate_pulses((100, 200, 300, 400)) == (100, 200, 300, 400)
    assert validate_pulses((100, 200, 300, 1001)) is None
