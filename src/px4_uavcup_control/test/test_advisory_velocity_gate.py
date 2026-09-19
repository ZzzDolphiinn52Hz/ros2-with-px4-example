import math

import pytest

from px4_uavcup_control.obstacle.advisory_velocity_gate import bounded_velocity


def test_velocity_inside_limits_is_unchanged():
    assert bounded_velocity(0.1, -0.1, 0.05, 0.2, 0.12) == pytest.approx(
        (0.1, -0.1, 0.05))


def test_horizontal_limit_preserves_direction():
    assert bounded_velocity(0.3, 0.4, 0.0, 0.2, 0.12) == pytest.approx(
        (0.12, 0.16, 0.0))


def test_vertical_limit_clamps_both_directions():
    assert bounded_velocity(0.0, 0.0, 0.3, 0.2, 0.12)[2] == 0.12
    assert bounded_velocity(0.0, 0.0, -0.3, 0.2, 0.12)[2] == -0.12


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_non_finite_velocity_is_rejected(value):
    with pytest.raises(ValueError):
        bounded_velocity(value, 0.0, 0.0, 0.2, 0.12)
