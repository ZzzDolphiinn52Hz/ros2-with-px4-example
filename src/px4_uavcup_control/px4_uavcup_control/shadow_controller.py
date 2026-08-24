"""Pure, testable shadow controller for three-sector depth summaries."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Deque, Optional, Tuple

import numpy as np


class AvoidanceState(str, Enum):
    CLEAR = 'CLEAR'
    AVOID_LEFT = 'AVOID_LEFT'
    AVOID_RIGHT = 'AVOID_RIGHT'
    BRAKE = 'BRAKE'
    FAILSAFE = 'FAILSAFE'


@dataclass(frozen=True)
class ControllerConfig:
    emergency_distance_m: float = 0.35
    avoid_enter_distance_m: float = 0.45
    clear_exit_distance_m: float = 0.50
    minimum_valid_fraction: float = 0.25
    forward_speed_mps: float = 0.4
    avoidance_forward_speed_mps: float = 0.1
    lateral_speed_mps: float = 0.3
    median_window: int = 5
    ema_alpha: float = 0.35
    recovery_frames: int = 3
    minimum_direction_hold_sec: float = 0.5
    side_switch_margin_m: float = 0.25

    def validate(self) -> None:
        if not (
                0.0 < self.emergency_distance_m
                < self.avoid_enter_distance_m
                < self.clear_exit_distance_m):
            raise ValueError(
                'Distance thresholds must satisfy emergency < avoid < clear')
        if not 0.0 <= self.minimum_valid_fraction <= 1.0:
            raise ValueError('minimum_valid_fraction must be in [0, 1]')
        if min(
                self.forward_speed_mps,
                self.avoidance_forward_speed_mps,
                self.lateral_speed_mps) < 0.0:
            raise ValueError('Advisory speeds cannot be negative')
        if self.median_window <= 0:
            raise ValueError('median_window must be positive')
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError('ema_alpha must be in (0, 1]')
        if self.recovery_frames <= 0:
            raise ValueError('recovery_frames must be positive')
        if self.minimum_direction_hold_sec < 0.0:
            raise ValueError('minimum_direction_hold_sec cannot be negative')
        if self.side_switch_margin_m < 0.0:
            raise ValueError('side_switch_margin_m cannot be negative')


@dataclass(frozen=True)
class ControllerDecision:
    state: AvoidanceState
    forward_mps: float
    left_mps: float
    reason: str
    filtered_left_m: float = float('nan')
    filtered_center_m: float = float('nan')
    filtered_right_m: float = float('nan')


class ThreeSectorFilter:
    """Median filter followed by EMA for left, centre and right sectors."""

    def __init__(self, window: int, alpha: float) -> None:
        if window <= 0 or not 0.0 < alpha <= 1.0:
            raise ValueError('Invalid filter configuration')
        self._samples: Tuple[Deque[float], Deque[float], Deque[float]] = (
            deque(maxlen=window),
            deque(maxlen=window),
            deque(maxlen=window),
        )
        self._alpha = float(alpha)
        self._ema: Optional[Tuple[float, float, float]] = None

    def reset(self) -> None:
        for samples in self._samples:
            samples.clear()
        self._ema = None

    def update(
            self,
            left_m: float,
            center_m: float,
            right_m: float) -> Tuple[float, float, float]:
        values = (float(left_m), float(center_m), float(right_m))
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('Sector distances must be finite and positive')
        for samples, value in zip(self._samples, values):
            samples.append(value)
        medians = tuple(float(np.median(samples)) for samples in self._samples)
        if self._ema is None:
            self._ema = medians
        else:
            self._ema = tuple(
                self._alpha * median + (1.0 - self._alpha) * previous
                for median, previous in zip(medians, self._ema)
            )
        return self._ema


class ShadowController:
    """Generate advisory body-FLU velocities without commanding PX4."""

    def __init__(self, config: ControllerConfig) -> None:
        config.validate()
        self.config = config
        self._filter = ThreeSectorFilter(
            config.median_window, config.ema_alpha)
        self._state = AvoidanceState.FAILSAFE
        self._state_started = 0.0
        self._recovery_count = 0

    @property
    def state(self) -> AvoidanceState:
        return self._state

    def invalidate(self, reason: str, now_sec: float) -> ControllerDecision:
        self._filter.reset()
        self._recovery_count = 0
        self._set_state(AvoidanceState.FAILSAFE, now_sec)
        return ControllerDecision(
            state=AvoidanceState.FAILSAFE,
            forward_mps=0.0,
            left_mps=0.0,
            reason=reason,
        )

    def update(
            self,
            left_m: float,
            center_m: float,
            right_m: float,
            valid_fraction: float,
            now_sec: float) -> ControllerDecision:
        values = (left_m, center_m, right_m)
        if (
                not math.isfinite(float(valid_fraction))
                or valid_fraction < self.config.minimum_valid_fraction
                or not all(
                    math.isfinite(float(value)) and value > 0.0
                    for value in values)):
            return self.invalidate('invalid depth summary', now_sec)

        filtered = self._filter.update(*values)
        if self._state == AvoidanceState.FAILSAFE:
            self._recovery_count += 1
            if self._recovery_count < self.config.recovery_frames:
                return self._decision(
                    AvoidanceState.FAILSAFE,
                    f'recovering {self._recovery_count}/'
                    f'{self.config.recovery_frames}',
                    filtered,
                )

        left, center, right = filtered
        nearest = min(filtered)
        if nearest < self.config.emergency_distance_m:
            return self._transition(
                AvoidanceState.BRAKE,
                'emergency clearance threshold',
                filtered,
                now_sec,
            )

        if self._state in (
                AvoidanceState.AVOID_LEFT,
                AvoidanceState.AVOID_RIGHT):
            if (
                    center >= self.config.clear_exit_distance_m
                    and nearest >= self.config.avoid_enter_distance_m):
                return self._transition(
                    AvoidanceState.CLEAR,
                    'path clear with hysteresis',
                    filtered,
                    now_sec,
                )
            selected = self._select_direction(left, right, now_sec)
            if selected is None:
                return self._transition(
                    AvoidanceState.BRAKE,
                    'no lateral clearance',
                    filtered,
                    now_sec,
                )
            return self._transition(
                selected,
                'maintaining avoidance direction',
                filtered,
                now_sec,
            )

        if self._state in (
                AvoidanceState.FAILSAFE,
                AvoidanceState.BRAKE):
            if (
                    center >= self.config.clear_exit_distance_m
                    and nearest >= self.config.avoid_enter_distance_m):
                return self._transition(
                    AvoidanceState.CLEAR,
                    'path clear with hysteresis',
                    filtered,
                    now_sec,
                )
            selected = self._select_direction(left, right, now_sec)
            if selected is None:
                return self._transition(
                    AvoidanceState.BRAKE,
                    'waiting for clear-path threshold',
                    filtered,
                    now_sec,
                )
            return self._transition(
                selected,
                'forward clearance below clear threshold',
                filtered,
                now_sec,
            )

        if center < self.config.avoid_enter_distance_m:
            selected = self._select_direction(left, right, now_sec)
            if selected is None:
                return self._transition(
                    AvoidanceState.BRAKE,
                    'centre and both sides blocked',
                    filtered,
                    now_sec,
                )
            return self._transition(
                selected,
                'centre blocked',
                filtered,
                now_sec,
            )

        return self._transition(
            AvoidanceState.CLEAR,
            'forward path clear',
            filtered,
            now_sec,
        )

    def _select_direction(
            self,
            left_m: float,
            right_m: float,
            now_sec: float) -> Optional[AvoidanceState]:
        threshold = self.config.avoid_enter_distance_m
        if max(left_m, right_m) < threshold:
            return None

        held_long_enough = (
            now_sec - self._state_started
            >= self.config.minimum_direction_hold_sec)
        if self._state == AvoidanceState.AVOID_LEFT:
            if (
                    left_m >= threshold
                    and (
                        not held_long_enough
                        or left_m + self.config.side_switch_margin_m
                        >= right_m)):
                return AvoidanceState.AVOID_LEFT
        elif self._state == AvoidanceState.AVOID_RIGHT:
            if (
                    right_m >= threshold
                    and (
                        not held_long_enough
                        or right_m + self.config.side_switch_margin_m
                        >= left_m)):
                return AvoidanceState.AVOID_RIGHT

        if left_m >= right_m and left_m >= threshold:
            return AvoidanceState.AVOID_LEFT
        if right_m >= threshold:
            return AvoidanceState.AVOID_RIGHT
        return None

    def _transition(
            self,
            state: AvoidanceState,
            reason: str,
            filtered: Tuple[float, float, float],
            now_sec: float) -> ControllerDecision:
        self._set_state(state, now_sec)
        return self._decision(state, reason, filtered)

    def _set_state(self, state: AvoidanceState, now_sec: float) -> None:
        if state != self._state:
            self._state = state
            self._state_started = float(now_sec)

    def _decision(
            self,
            state: AvoidanceState,
            reason: str,
            filtered: Tuple[float, float, float]) -> ControllerDecision:
        forward = 0.0
        left = 0.0
        if state == AvoidanceState.CLEAR:
            forward = self.config.forward_speed_mps
        elif state == AvoidanceState.AVOID_LEFT:
            forward = self.config.avoidance_forward_speed_mps
            left = self.config.lateral_speed_mps
        elif state == AvoidanceState.AVOID_RIGHT:
            forward = self.config.avoidance_forward_speed_mps
            left = -self.config.lateral_speed_mps
        return ControllerDecision(
            state=state,
            forward_mps=forward,
            left_mps=left,
            reason=reason,
            filtered_left_m=filtered[0],
            filtered_center_m=filtered[1],
            filtered_right_m=filtered[2],
        )
