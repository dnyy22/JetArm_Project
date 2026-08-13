from tool_frame_adapter import ToolFrameAdapter, URDF_SOLDER_TIP_IN_GRIPPER_M


def test_urdf_solder_tip_origin_is_derived_from_fixed_joints():
    assert URDF_SOLDER_TIP_IN_GRIPPER_M == (-0.0014, 0.0, 0.16175)


def test_tip_and_legacy_conversion_are_inverses_at_any_pitch():
    frames = ToolFrameAdapter((0.020, 0.010, 0.120))
    tip = (0.22, -0.08, 0.15)
    for pitch in (-60.0, 0.0, 45.0, 80.0):
        legacy = frames.tip_to_legacy(tip, pitch)
        recovered = frames.legacy_to_tip(legacy, pitch)
        assert all(abs(actual - expected) < 1e-12 for actual, expected in zip(recovered, tip))
