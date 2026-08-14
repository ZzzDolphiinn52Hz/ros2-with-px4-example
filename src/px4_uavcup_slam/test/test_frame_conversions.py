import math

import numpy as np
import pytest

from px4_uavcup_slam.gz_lidar_bridge import px4_quat_tilt_rad
from px4_uavcup_slam.px4_odom_tf import (
    ned_xy_reset_to_enu_continuity_offset,
    ned_to_enu_position,
    px4_quat_ned_frd_to_enu_flu,
    quaternion_xyzw_multiply,
    quaternion_xyzw_to_yaw,
)


def _yaw_from_xyzw(q):
    x, y, z, w = q
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _angle_error(actual, expected):
    return math.atan2(
        math.sin(actual - expected),
        math.cos(actual - expected),
    )


@pytest.mark.parametrize('heading_deg', [0.0, 30.0, 90.0, 145.0, 180.0])
def test_px4_heading_becomes_ros_enu_yaw(heading_deg):
    heading = math.radians(heading_deg)
    q_ned_frd = np.array([
        math.cos(heading / 2.0),
        0.0,
        0.0,
        math.sin(heading / 2.0),
    ])

    q_enu_flu = px4_quat_ned_frd_to_enu_flu(q_ned_frd)
    actual = _yaw_from_xyzw(q_enu_flu)
    expected = math.pi / 2.0 - heading

    assert _angle_error(actual, expected) == pytest.approx(0.0, abs=1e-9)


def test_ned_position_becomes_enu():
    assert ned_to_enu_position(1.0, 2.0, -3.0) == (2.0, 1.0, 3.0)


def test_px4_xy_reset_is_cancelled_in_enu():
    raw_before_ned = np.array([10.0, 20.0])
    delta_ned = np.array([1.5, -2.0])
    raw_after_ned = raw_before_ned + delta_ned
    offset_enu = np.array(ned_xy_reset_to_enu_continuity_offset(*delta_ned))

    enu_before = np.array([raw_before_ned[1], raw_before_ned[0]])
    enu_after_corrected = np.array([
        raw_after_ned[1], raw_after_ned[0]
    ]) + offset_enu

    assert enu_after_corrected == pytest.approx(enu_before)


def test_tilt_ignores_yaw_and_detects_roll():
    yaw = math.radians(70.0)
    assert px4_quat_tilt_rad([
        math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)
    ]) == pytest.approx(0.0)

    roll = math.radians(12.0)
    assert px4_quat_tilt_rad([
        math.cos(roll / 2.0), math.sin(roll / 2.0), 0.0, 0.0
    ]) == pytest.approx(roll)


def test_planar_yaw_and_residual_tilt_recompose_full_attitude():
    yaw = math.radians(35.0)
    roll = math.radians(8.0)
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
    q_roll = np.array([math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0)])
    q_full = quaternion_xyzw_multiply(q_yaw, q_roll)

    actual_yaw = quaternion_xyzw_to_yaw(q_full)
    q_yaw_inverse = np.array([0.0, 0.0, -q_yaw[2], q_yaw[3]])
    q_residual = quaternion_xyzw_multiply(q_yaw_inverse, q_full)
    recomposed = quaternion_xyzw_multiply(q_yaw, q_residual)

    assert _angle_error(actual_yaw, yaw) == pytest.approx(0.0, abs=1e-9)
    assert recomposed == pytest.approx(q_full, abs=1e-9)
