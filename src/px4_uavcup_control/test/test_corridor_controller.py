import math

import pytest

from px4_uavcup_control.obstacle.corridor_controller import (
    CorridorController,
    CorridorControllerConfig,
    CorridorState,
)


def config(**overrides):
    values = dict(recovery_frames=1, ema_alpha=1.0)
    values.update(overrides)
    return CorridorControllerConfig(**values)


def test_upper_left_corridor_advises_left_and_up():
    controller = CorridorController(config())

    decision = controller.update(-0.6, -0.4, 0.32, 0.32, 0.8, 1.0)

    assert decision.state == CorridorState.TRACK
    assert decision.forward_mps > 0.0
    assert decision.left_mps > 0.0
    assert decision.up_mps > 0.0


def test_lower_right_corridor_advises_right_and_down():
    controller = CorridorController(config())

    decision = controller.update(0.5, 0.3, 0.32, 0.32, 0.8, 1.0)

    assert decision.left_mps < 0.0
    assert decision.up_mps < 0.0


def test_centered_corridor_keeps_lateral_and_vertical_zero():
    controller = CorridorController(config())

    decision = controller.update(0.02, -0.03, 0.32, 0.32, 0.8, 1.0)

    assert decision.state == CorridorState.CENTERED
    assert decision.left_mps == 0.0
    assert decision.up_mps == 0.0


def test_invalid_or_low_clearance_stops_motion():
    invalid_controller = CorridorController(config())
    brake_controller = CorridorController(config())

    invalid = invalid_controller.update(
        math.nan, 0.0, 0.32, 0.32, 0.8, 1.0)
    brake = brake_controller.update(
        0.0, 0.0, 0.32, 0.32, 0.05, 1.0)

    assert invalid.state == CorridorState.FAILSAFE
    assert brake.state == CorridorState.BRAKE
    assert invalid.forward_mps == brake.forward_mps == 0.0


def test_target_step_limit_prevents_one_frame_direction_flip():
    controller = CorridorController(config(maximum_target_step=0.2))
    first = controller.update(-0.8, 0.0, 0.32, 0.32, 0.8, 1.0)
    second = controller.update(0.8, 0.0, 0.32, 0.32, 0.8, 1.0)

    assert first.left_mps > 0.0
    assert second.left_mps > 0.0
    assert second.target_x_normalized == pytest.approx(-0.6)
