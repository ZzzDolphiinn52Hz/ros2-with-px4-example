import math

import numpy as np

from px4_uavcup_control.obstacle.shadow_controller import (
    AvoidanceState,
    ControllerConfig,
    ShadowController,
    ThreeSectorFilter,
)


def config(**overrides):
    values = dict(
        median_window=1,
        ema_alpha=1.0,
        recovery_frames=1,
        minimum_direction_hold_sec=0.5,
    )
    values.update(overrides)
    return ControllerConfig(**values)


def test_clear_path_advises_forward_motion():
    controller = ShadowController(config())

    decision = controller.update(2.0, 2.0, 2.0, 1.0, 0.0)

    assert decision.state == AvoidanceState.CLEAR
    assert decision.forward_mps == 0.4
    assert decision.left_mps == 0.0


def test_center_obstacle_chooses_side_with_more_clearance():
    controller = ShadowController(config())

    decision = controller.update(0.8, 0.4, 0.6, 1.0, 0.0)

    assert decision.state == AvoidanceState.AVOID_LEFT
    assert decision.forward_mps == 0.1
    assert decision.left_mps == 0.3


def test_right_avoidance_uses_negative_flu_lateral_velocity():
    controller = ShadowController(config())

    decision = controller.update(0.6, 0.4, 0.8, 1.0, 0.0)

    assert decision.state == AvoidanceState.AVOID_RIGHT
    assert decision.left_mps == -0.3


def test_emergency_clearance_brakes_even_if_center_is_clear():
    controller = ShadowController(config())

    decision = controller.update(0.3, 2.0, 2.0, 1.0, 0.0)

    assert decision.state == AvoidanceState.BRAKE
    assert decision.forward_mps == 0.0
    assert decision.left_mps == 0.0


def test_center_and_both_sides_blocked_brakes():
    controller = ShadowController(config())

    decision = controller.update(0.40, 0.40, 0.42, 1.0, 0.0)

    assert decision.state == AvoidanceState.BRAKE


def test_invalid_input_fails_safe_and_requires_stable_recovery():
    controller = ShadowController(ShadowControllerTestConfig.recovery_three())
    controller.update(2.0, 2.0, 2.0, 1.0, 0.0)
    controller.update(2.0, 2.0, 2.0, 1.0, 0.1)
    controller.update(2.0, 2.0, 2.0, 1.0, 0.2)

    invalid = controller.update(math.nan, 2.0, 2.0, 1.0, 0.3)
    recovering_one = controller.update(2.0, 2.0, 2.0, 1.0, 0.4)
    recovering_two = controller.update(2.0, 2.0, 2.0, 1.0, 0.5)
    recovered = controller.update(2.0, 2.0, 2.0, 1.0, 0.6)

    assert invalid.state == AvoidanceState.FAILSAFE
    assert recovering_one.state == AvoidanceState.FAILSAFE
    assert recovering_two.state == AvoidanceState.FAILSAFE
    assert recovered.state == AvoidanceState.CLEAR


def test_direction_hold_prevents_immediate_left_right_oscillation():
    controller = ShadowController(config())
    first = controller.update(0.8, 0.4, 0.6, 1.0, 0.0)

    held = controller.update(0.5, 0.4, 0.9, 1.0, 0.1)
    switched = controller.update(0.5, 0.4, 0.9, 1.0, 0.6)

    assert first.state == AvoidanceState.AVOID_LEFT
    assert held.state == AvoidanceState.AVOID_LEFT
    assert switched.state == AvoidanceState.AVOID_RIGHT


def test_startup_requires_half_metre_before_clear():
    controller = ShadowController(config())

    below = controller.update(0.8, 0.48, 0.7, 1.0, 0.0)
    clear = controller.update(0.8, 0.51, 0.7, 1.0, 0.1)

    assert below.state != AvoidanceState.CLEAR
    assert clear.state == AvoidanceState.CLEAR


def test_filter_rejects_one_frame_distance_spike():
    distance_filter = ThreeSectorFilter(window=3, alpha=1.0)

    distance_filter.update(2.0, 2.0, 2.0)
    distance_filter.update(2.0, 2.0, 2.0)
    filtered = distance_filter.update(0.4, 0.4, 0.4)

    np.testing.assert_allclose(filtered, (2.0, 2.0, 2.0))


def test_relative_thresholds_match_measured_clear_and_obstacle_scenes():
    relative = config(
        emergency_distance_m=0.001,
        avoid_enter_distance_m=0.05,
        clear_exit_distance_m=0.10,
    )

    clear = ShadowController(relative).update(
        0.175, 0.118, 0.064, 1.0, 0.0)
    center_obstacle = ShadowController(relative).update(
        0.772, 0.015, 0.082, 1.0, 0.0)
    left_obstacle = ShadowController(relative).update(
        0.035, 0.340, 0.558, 1.0, 0.0)

    assert clear.state == AvoidanceState.CLEAR
    assert center_obstacle.state == AvoidanceState.AVOID_LEFT
    assert left_obstacle.state == AvoidanceState.AVOID_RIGHT


class ShadowControllerTestConfig:
    @staticmethod
    def recovery_three():
        return config(recovery_frames=3)
