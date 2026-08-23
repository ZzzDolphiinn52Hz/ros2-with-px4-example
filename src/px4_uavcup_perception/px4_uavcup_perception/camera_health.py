"""Camera-image health checks used before monocular depth inference."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraHealth:
    """Quality metrics and validity decision for one camera frame."""

    healthy: bool
    reason: str
    brightness_mean: float
    contrast_stddev: float
    gradient_mean: float
    dark_fraction: float
    bright_fraction: float


def assess_camera_health(
        bgr_image: np.ndarray,
        sample_stride: int = 4,
        minimum_brightness: float = 8.0,
        maximum_brightness: float = 247.0,
        minimum_contrast_stddev: float = 6.0,
        minimum_gradient_mean: float = 2.0,
        maximum_dark_fraction: float = 0.98,
        maximum_bright_fraction: float = 0.98) -> CameraHealth:
    """Detect covered, saturated and textureless camera frames.

    The check intentionally samples the image to keep its cost small compared
    with TensorRT inference. Low contrast and low gradient must occur together
    before a normally exposed frame is rejected, which reduces false alarms on
    plain walls.
    """
    image = np.asarray(bgr_image)
    if image.ndim != 3 or image.shape[2] < 3 or image.size == 0:
        raise ValueError('Camera image must be a non-empty BGR array')
    if sample_stride <= 0:
        raise ValueError('Camera-health sample stride must be positive')
    if not 0.0 <= maximum_dark_fraction <= 1.0:
        raise ValueError('maximum_dark_fraction must be between 0 and 1')
    if not 0.0 <= maximum_bright_fraction <= 1.0:
        raise ValueError('maximum_bright_fraction must be between 0 and 1')

    sampled = image[::sample_stride, ::sample_stride, :3].astype(
        np.float32, copy=False)
    gray = (
        0.114 * sampled[:, :, 0]
        + 0.587 * sampled[:, :, 1]
        + 0.299 * sampled[:, :, 2]
    )
    brightness = float(np.mean(gray))
    contrast = float(np.std(gray))
    dark_fraction = float(np.mean(gray <= 10.0))
    bright_fraction = float(np.mean(gray >= 245.0))

    horizontal_gradient = (
        np.mean(np.abs(np.diff(gray, axis=1)))
        if gray.shape[1] > 1 else 0.0)
    vertical_gradient = (
        np.mean(np.abs(np.diff(gray, axis=0)))
        if gray.shape[0] > 1 else 0.0)
    gradient = float(
        0.5 * (horizontal_gradient + vertical_gradient))

    reason = 'valid'
    if (
            brightness <= minimum_brightness
            or dark_fraction >= maximum_dark_fraction):
        reason = 'camera too dark or covered'
    elif (
            brightness >= maximum_brightness
            or bright_fraction >= maximum_bright_fraction):
        reason = 'camera overexposed or covered'
    elif (
            contrast < minimum_contrast_stddev
            and gradient < minimum_gradient_mean):
        reason = 'camera image has insufficient texture'

    return CameraHealth(
        healthy=reason == 'valid',
        reason=reason,
        brightness_mean=brightness,
        contrast_stddev=contrast,
        gradient_mean=gradient,
        dark_fraction=dark_fraction,
        bright_fraction=bright_fraction,
    )


class CameraHealthGate:
    """Debounce persistent failures and require stable recovery."""

    def __init__(
            self,
            failure_frames: int = 3,
            recovery_frames: int = 5) -> None:
        if failure_frames <= 0 or recovery_frames <= 0:
            raise ValueError('Health-gate frame counts must be positive')
        self.failure_frames = int(failure_frames)
        self.recovery_frames = int(recovery_frames)
        self.healthy = True
        self.failure_count = 0
        self.recovery_count = 0

    def update(self, observation_healthy: bool) -> bool:
        """Update and return the debounced camera-health state."""
        if self.healthy:
            self.recovery_count = 0
            if observation_healthy:
                self.failure_count = 0
            else:
                self.failure_count += 1
                if self.failure_count >= self.failure_frames:
                    self.healthy = False
                    self.failure_count = 0
        else:
            self.failure_count = 0
            if observation_healthy:
                self.recovery_count += 1
                if self.recovery_count >= self.recovery_frames:
                    self.healthy = True
                    self.recovery_count = 0
            else:
                self.recovery_count = 0
        return self.healthy
