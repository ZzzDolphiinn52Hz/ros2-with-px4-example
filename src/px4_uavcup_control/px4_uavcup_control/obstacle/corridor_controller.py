"""Pure shadow controller for a 2D relative-depth corridor."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class CorridorState(str, Enum):
    TRACK = 'TRACK_CORRIDOR'
    CENTERED = 'CORRIDOR_CENTERED'
    BRAKE = 'BRAKE'
    FAILSAFE = 'FAILSAFE'


@dataclass(frozen=True)
class CorridorControllerConfig:
    minimum_clearance: float = 0.10
    minimum_valid_fraction: float = 0.90
    target_deadband: float = 0.08
    forward_speed_mps: float = 0.20
    minimum_forward_scale: float = 0.25
    maximum_lateral_speed_mps: float = 0.20
    maximum_vertical_speed_mps: float = 0.12
    ema_alpha: float = 0.35
    maximum_target_step: float = 0.20
    recovery_frames: int = 3

    def validate(self) -> None:
        for name, value in (
                ('minimum_clearance', self.minimum_clearance),
                ('minimum_valid_fraction', self.minimum_valid_fraction),
                ('target_deadband', self.target_deadband),
                ('minimum_forward_scale', self.minimum_forward_scale),
                ('ema_alpha', self.ema_alpha),
                ('maximum_target_step', self.maximum_target_step)):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must be in [0, 1]')
        for name, value in (
                ('forward_speed_mps', self.forward_speed_mps),
                ('maximum_lateral_speed_mps',
                 self.maximum_lateral_speed_mps),
                ('maximum_vertical_speed_mps',
                 self.maximum_vertical_speed_mps)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and non-negative')
        if self.recovery_frames <= 0:
            raise ValueError('recovery_frames must be positive')


@dataclass(frozen=True)
class CorridorDecision:
    state: CorridorState
    forward_mps: float
    left_mps: float
    up_mps: float
    reason: str
    target_x_normalized: float = float('nan')
    target_y_normalized: float = float('nan')
    clearance: float = float('nan')


class CorridorController:
    """Turn one image-space opening into bounded body-FLU advisories."""

    def __init__(self, config: CorridorControllerConfig) -> None:
        config.validate()
        self.config = config
        self._target = None
        self._recovery_count = 0

    def invalidate(self, reason: str) -> CorridorDecision:
        self._target = None
        self._recovery_count = 0
        return CorridorDecision(
            state=CorridorState.FAILSAFE,
            forward_mps=0.0,
            left_mps=0.0,
            up_mps=0.0,
            reason=reason,
        )

    def update(
            self,
            x_normalized: float,
            y_normalized: float,
            width_fraction: float,
            height_fraction: float,
            clearance: float,
            valid_fraction: float) -> CorridorDecision:
        values = (
            x_normalized, y_normalized, width_fraction, height_fraction,
            clearance, valid_fraction)
        if not all(math.isfinite(float(value)) for value in values):
            return self.invalidate('invalid corridor')
        if (not -1.0 <= x_normalized <= 1.0
                or not -1.0 <= y_normalized <= 1.0
                or not 0.0 < width_fraction <= 1.0
                or not 0.0 < height_fraction <= 1.0
                or valid_fraction < self.config.minimum_valid_fraction):
            return self.invalidate('corridor geometry or coverage invalid')
        if not 0.0 <= clearance <= 1.0:
            return self.invalidate('corridor clearance invalid')

        self._recovery_count += 1
        if self._recovery_count < self.config.recovery_frames:
            return CorridorDecision(
                state=CorridorState.FAILSAFE,
                forward_mps=0.0,
                left_mps=0.0,
                up_mps=0.0,
                reason=(
                    f'recovering {self._recovery_count}/'
                    f'{self.config.recovery_frames}'),
                target_x_normalized=x_normalized,
                target_y_normalized=y_normalized,
                clearance=clearance,
            )
        if clearance < self.config.minimum_clearance:
            return CorridorDecision(
                state=CorridorState.BRAKE,
                forward_mps=0.0,
                left_mps=0.0,
                up_mps=0.0,
                reason='no corridor above clearance threshold',
                target_x_normalized=x_normalized,
                target_y_normalized=y_normalized,
                clearance=clearance,
            )

        target = np.array(
            [x_normalized, y_normalized], dtype=np.float64)
        if self._target is None:
            self._target = target
        else:
            delta = np.clip(
                target - self._target,
                -self.config.maximum_target_step,
                self.config.maximum_target_step)
            stepped = self._target + delta
            self._target = (
                self.config.ema_alpha * stepped
                + (1.0 - self.config.ema_alpha) * self._target)
        filtered_x = float(self._target[0])
        filtered_y = float(self._target[1])
        x_error = (
            0.0 if abs(filtered_x) < self.config.target_deadband
            else filtered_x)
        y_error = (
            0.0 if abs(filtered_y) < self.config.target_deadband
            else filtered_y)
        displacement = min(1.0, max(abs(filtered_x), abs(filtered_y)))
        forward_scale = max(
            self.config.minimum_forward_scale, 1.0 - displacement)
        forward = self.config.forward_speed_mps * forward_scale
        left = -x_error * self.config.maximum_lateral_speed_mps
        up = -y_error * self.config.maximum_vertical_speed_mps
        centred = x_error == 0.0 and y_error == 0.0
        return CorridorDecision(
            state=(CorridorState.CENTERED if centred
                   else CorridorState.TRACK),
            forward_mps=forward,
            left_mps=left,
            up_mps=up,
            reason=('corridor centered' if centred
                    else 'tracking 2D relative corridor'),
            target_x_normalized=filtered_x,
            target_y_normalized=filtered_y,
            clearance=clearance,
        )
