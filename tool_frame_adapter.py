"""Convert solder-tip waypoints to the legacy TCP used by the IK service.

The vendor IK service accepts only a position and a pitch.  Its TCP is part of
the compiled solver, so changing the physical end effector must be handled
before making the service request.  This module keeps that compensation in one
place and deliberately has no ROS dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from typing import Sequence


# From soldering_tool.urdf.xacro:
# T_gripper_tool = Trans(-0.0014, 0, 0.0585) Rz(0) Ry(-pi/2) Rx(pi/2)
# T_tool_tip = Trans(0.10325, 0, 0)
# Therefore the solder-tip origin in gripper_servo_link is:
URDF_SOLDER_TIP_IN_GRIPPER_M = (-0.0014, 0.0, 0.16175)

# ``driver/kinematics/kinematics/transform.py`` in the vendor stack defines
# the legacy TCP length as ``link3 + tool_link``.  It is the best available
# default for the opaque solver and can be overridden after calibration.
VENDOR_LEGACY_TCP_IN_GRIPPER_M = (0.0, 0.0, 0.16645583202)


def _vector(value: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    return tuple(float(component) for component in value)  # type: ignore[return-value]


def _rotate_y(vector: Sequence[float], pitch_degrees: float) -> tuple[float, float, float]:
    """Rotate a legacy-TCP local vector into the world frame.

    This follows the convention used by ``GetRobotPose`` in the executor: its
    one orientational degree of freedom is the Y-axis pitch.  Roll/yaw cannot
    be represented by the legacy five-axis IK API and are intentionally not
    invented here.
    """
    x, y, z = _vector(vector, "vector")
    angle = radians(pitch_degrees)
    return (cos(angle) * x + sin(angle) * z, y, -sin(angle) * x + cos(angle) * z)


@dataclass(frozen=True)
class ToolFrameAdapter:
    """Static transform between the legacy IK TCP and the solder-tip TCP.

    Both origins are expressed in ``gripper_servo_link``.  The legacy origin
    must be measured or taken from the legacy robot description / IK source;
    it is not present in this repository.  Positions supplied to this class
    are metres in the robot base frame.
    """

    legacy_tcp_in_gripper_m: tuple[float, float, float]
    solder_tip_in_gripper_m: tuple[float, float, float] = URDF_SOLDER_TIP_IN_GRIPPER_M

    @property
    def legacy_to_tip_m(self) -> tuple[float, float, float]:
        """Translation from old TCP to new tip in the legacy model's zero-pitch frame.

        The two frame origins are rigidly attached to the same gripper frame.
        The current IK interface has only pitch, so this is the translational
        part of ``inv(T_gripper_legacy) @ T_gripper_tip`` under the vendor
        model's convention that its zero-pitch TCP axes align with the gripper
        reference axes.
        """
        return tuple(
            tip - legacy
            for tip, legacy in zip(self.solder_tip_in_gripper_m, self.legacy_tcp_in_gripper_m)
        )  # type: ignore[return-value]

    def tip_to_legacy(self, tip_xyz_m: Sequence[float], pitch_degrees: float) -> list[float]:
        """Return the old-TCP target that places the new tip at ``tip_xyz_m``.

        ``p_world_legacy = p_world_tip - R_y(pitch) p_legacy_tip``.
        This is the target passed to the unmodified ``.so`` IK service.
        """
        tip = _vector(tip_xyz_m, "tip_xyz_m")
        dx, dy, dz = _rotate_y(self.legacy_to_tip_m, pitch_degrees)
        return [tip[0] - dx, tip[1] - dy, tip[2] - dz]

    def legacy_to_tip(self, legacy_xyz_m: Sequence[float], pitch_degrees: float) -> list[float]:
        """Convert a pose reported by the old IK TCP into solder-tip position."""
        legacy = _vector(legacy_xyz_m, "legacy_xyz_m")
        dx, dy, dz = _rotate_y(self.legacy_to_tip_m, pitch_degrees)
        return [legacy[0] + dx, legacy[1] + dy, legacy[2] + dz]
