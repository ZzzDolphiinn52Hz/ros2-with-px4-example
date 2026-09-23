import numpy as np
import pytest

from px4_uavcup_control.landing.landing_state import (
    LandingState,
    approach_height_m,
    lock_state_from_frames,
    on_marker_stale,
    publishes_command,
    uses_marker,
)
from px4_uavcup_control.landing.pid import (
    DOWN_CAMERA_OPTICAL_TO_BODY_FLU,
    PidAxis,
    camera_target_to_body_flu,
    gripper_clearance_from_marker,
    ready_for_blind_descent,
)


def test_down_camera_axes_match_rear_facing_mounting():
    # The installed camera is yawed 180 degrees: image-bottom is ahead and
    # image-right is vehicle-left.
    ahead = camera_target_to_body_flu(
        [0.0, 0.2, 1.0], DOWN_CAMERA_OPTICAL_TO_BODY_FLU)
    right = camera_target_to_body_flu(
        [-0.2, 0.0, 1.0], DOWN_CAMERA_OPTICAL_TO_BODY_FLU)
    np.testing.assert_allclose(ahead, [0.2, 0.0, -1.0])
    np.testing.assert_allclose(right, [0.0, -0.2, -1.0])


def test_rear_camera_lever_arm_centres_marker_under_gripper():
    camera_position = [-0.10, 0.0, 0.0]

    camera_over_marker = camera_target_to_body_flu(
        [0.0, 0.0, 1.0],
        DOWN_CAMERA_OPTICAL_TO_BODY_FLU,
        camera_position)
    gripper_over_marker = camera_target_to_body_flu(
        [0.0, 0.10, 1.0],
        DOWN_CAMERA_OPTICAL_TO_BODY_FLU,
        camera_position)

    np.testing.assert_allclose(camera_over_marker, [-0.10, 0.0, -1.0])
    np.testing.assert_allclose(gripper_over_marker, [0.0, 0.0, -1.0])


def test_gripper_clearance_subtracts_vertical_camera_offset():
    assert gripper_clearance_from_marker(0.45, 0.15) == pytest.approx(0.30)


def test_blind_descent_gate_requires_both_height_and_alignment():
    assert ready_for_blind_descent(0.30, 0.05, 0.30, 0.08)
    assert not ready_for_blind_descent(0.31, 0.05, 0.30, 0.08)
    assert not ready_for_blind_descent(0.30, 0.09, 0.30, 0.08)


def test_pid_is_bounded_and_resettable():
    pid = PidAxis(1.0, 1.0, 0.0, integral_limit=0.1, output_limit=0.2)
    assert pid.update(1.0, 1.0) == pytest.approx(0.2)
    assert pid.integral == pytest.approx(0.1)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.previous_error is None


def test_pid_rejects_invalid_dt():
    pid = PidAxis(1.0, 0.0, 0.0, integral_limit=1.0, output_limit=1.0)
    with pytest.raises(ValueError):
        pid.update(1.0, 0.0)


def test_marker_loss_aborts_visual_approach_but_not_blind_descent():
    assert on_marker_stale(LandingState.VISUAL_APPROACH) is LandingState.ABORT
    assert on_marker_stale(LandingState.LOCK) is LandingState.SEARCH
    assert on_marker_stale(LandingState.SEARCH) is LandingState.SEARCH
    assert on_marker_stale(
        LandingState.BLIND_FINAL_DESCENT) is LandingState.BLIND_FINAL_DESCENT
    assert on_marker_stale(LandingState.COMPLETE) is LandingState.COMPLETE


def test_lock_requires_consecutive_frames_before_visual_approach():
    assert lock_state_from_frames(0, 5) is LandingState.SEARCH
    assert lock_state_from_frames(3, 5) is LandingState.LOCK
    assert lock_state_from_frames(5, 5) is LandingState.VISUAL_APPROACH


def test_blind_descent_does_not_use_marker():
    assert uses_marker(LandingState.VISUAL_APPROACH)
    assert not uses_marker(LandingState.BLIND_FINAL_DESCENT)
    assert not publishes_command(LandingState.IDLE)
    assert publishes_command(LandingState.ABORT)


def test_approach_height_prefers_the_higher_of_aruco_and_rangefinder():
    assert approach_height_m(0.28) == pytest.approx(0.28)
    assert approach_height_m(0.28, 0.40, True) == pytest.approx(0.40)
    assert approach_height_m(0.28, 0.20, True) == pytest.approx(0.28)
