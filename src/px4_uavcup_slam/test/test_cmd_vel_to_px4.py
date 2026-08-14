import math

import pytest

from px4_uavcup_slam.cmd_vel_to_px4 import (
    body_flu_to_ned_velocity,
    clamp_xy,
    px4_quaternion_to_heading_ned,
    ros_yaw_rate_to_ned,
    slew_xy,
    vehicle_command_result_text,
)
from px4_msgs.msg import VehicleCommandAck


def test_forward_at_north_heading_becomes_north_velocity():
    north, east = body_flu_to_ned_velocity(0.4, 0.0, 0.0)
    assert north == pytest.approx(0.4)
    assert east == pytest.approx(0.0, abs=1e-12)


def test_forward_at_east_heading_becomes_east_velocity():
    north, east = body_flu_to_ned_velocity(0.4, 0.0, math.pi / 2.0)
    assert north == pytest.approx(0.0, abs=1e-12)
    assert east == pytest.approx(0.4)


def test_ros_left_at_north_heading_becomes_west_velocity():
    north, east = body_flu_to_ned_velocity(0.0, 0.3, 0.0)
    assert north == pytest.approx(0.0, abs=1e-12)
    assert east == pytest.approx(-0.3)


def test_ros_positive_yaw_rate_changes_sign_for_ned():
    assert ros_yaw_rate_to_ned(0.25) == pytest.approx(-0.25)


@pytest.mark.parametrize('heading', [0.0, 0.7, math.pi / 2.0, -2.4])
def test_px4_attitude_quaternion_yields_ned_heading(heading):
    q = [math.cos(heading / 2.0), 0.0, 0.0, math.sin(heading / 2.0)]
    assert px4_quaternion_to_heading_ned(q) == pytest.approx(heading)


def test_px4_attitude_heading_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        px4_quaternion_to_heading_ned([0.0, 0.0, 0.0, 0.0])


def test_vehicle_command_result_has_readable_name():
    assert vehicle_command_result_text(
        VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
    ) == 'ACCEPTED'
    assert vehicle_command_result_text(99) == 'UNKNOWN(99)'


def test_clamp_xy_preserves_direction():
    x, y = clamp_xy(3.0, 4.0, 0.5)
    assert x == pytest.approx(0.3)
    assert y == pytest.approx(0.4)


def test_slew_xy_limits_vector_delta():
    x, y = slew_xy(0.0, 0.0, 0.3, 0.4, 0.1)
    assert x == pytest.approx(0.06)
    assert y == pytest.approx(0.08)


def test_slew_xy_reaches_near_target_without_overshoot():
    assert slew_xy(0.0, 0.0, 0.03, 0.04, 0.1) == pytest.approx((0.03, 0.04))
