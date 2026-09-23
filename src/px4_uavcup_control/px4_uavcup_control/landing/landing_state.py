"""Landing phases from visual lock through latched blind descent."""

from __future__ import annotations

from enum import Enum
import math


class LandingState(str, Enum):
    IDLE = 'idle'
    SEARCH = 'search'
    LOCK = 'lock'
    VISUAL_APPROACH = 'visual_approach'
    BLIND_FINAL_DESCENT = 'blind_final_descent'
    COMPLETE = 'complete'
    ABORT = 'abort'


def on_marker_stale(state: LandingState) -> LandingState:
    """Hold if the marker vanishes before XY is latched; ignore it after."""
    if state is LandingState.LOCK:
        return LandingState.SEARCH
    if state is LandingState.VISUAL_APPROACH:
        return LandingState.ABORT
    return state


def lock_state_from_frames(
        consecutive_frames: int, required_frames: int) -> LandingState:
    """Advance SEARCH → LOCK → VISUAL_APPROACH from consecutive detections."""
    if required_frames <= 0:
        raise ValueError('required_frames must be positive')
    if consecutive_frames < 0:
        raise ValueError('consecutive_frames must be non-negative')
    if consecutive_frames >= required_frames:
        return LandingState.VISUAL_APPROACH
    if consecutive_frames > 0:
        return LandingState.LOCK
    return LandingState.SEARCH


def uses_marker(state: LandingState) -> bool:
    return state in (
        LandingState.SEARCH,
        LandingState.LOCK,
        LandingState.VISUAL_APPROACH,
    )


def publishes_command(state: LandingState) -> bool:
    return state is not LandingState.IDLE


def approach_height_m(
        gripper_clearance_m: float,
        dist_bottom_m: float = float('nan'),
        dist_bottom_valid: bool = False) -> float:
    """Height used to enter blind descent.

    ArUco gripper clearance is pad-relative. If PX4 ``dist_bottom`` is valid,
    take the more conservative (higher) of the two so the vehicle does not go
    blind while still high above the surface.
    """
    if not math.isfinite(gripper_clearance_m) or gripper_clearance_m < 0.0:
        raise ValueError('gripper clearance must be finite and non-negative')
    if not dist_bottom_valid:
        return float(gripper_clearance_m)
    if not math.isfinite(dist_bottom_m) or dist_bottom_m < 0.0:
        raise ValueError('dist_bottom must be finite and non-negative')
    return max(float(gripper_clearance_m), float(dist_bottom_m))
