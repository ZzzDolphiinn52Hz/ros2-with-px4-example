from time import monotonic

from geometry_msgs.msg import PoseStamped
from px4_uavcup_control.landing.aruco_pid_node import ArucoLandingPidNode
from px4_uavcup_control.landing.capture_clock import CaptureClock
from px4_uavcup_control.landing.landing_state import LandingState


def test_capture_age_rejects_a_monotonically_increasing_backlog():
    clock = CaptureClock()
    assert not clock.is_fresh(999_900_000_000, 1.0, 0.35)
    clock.update(1_000_000_000_000, 1.0)
    for stamp in (801_000_000_000, 802_000_000_000):
        assert not clock.is_fresh(stamp, 1.01, 0.35)
    assert clock.is_fresh(999_900_000_000, 1.01, 0.35)
    assert not clock.is_fresh(1_000_100_000_000, 1.01, 0.35)


def test_paused_clock_and_reset_do_not_validate_frames():
    clock = CaptureClock()
    clock.update(2_000_000_000, 1.0)
    clock.update(2_000_000_000, 1.4)
    assert not clock.is_fresh(2_000_000_000, 1.4, 0.35)
    clock.update(2_100_000_000, 1.5)
    assert clock.is_fresh(2_000_000_000, 1.5, 0.35)
    clock.update(0, 1.6)
    clock.update(100_000_000, 1.7)
    assert not clock.is_fresh(100_000_000, 1.7, 0.35)


def test_pid_does_not_refresh_watchdog_on_old_or_duplicate_pose():
    pid = ArucoLandingPidNode.__new__(ArucoLandingPidNode)
    pid._state = LandingState.SEARCH
    pid._marker_timeout = 0.35
    pid._last_pose_time = float('-inf')
    pid._last_capture_ns = 891_000_000_000
    pid._capture_clock = CaptureClock()
    pid._capture_clock.update(891_100_000_000, monotonic())
    pose = PoseStamped()
    for second in (692, 891):
        pose.header.stamp.sec = second
        pid._on_pose(pose)
        assert pid._last_pose_time == float('-inf')
