"""Pure camera geometry helpers shared by hardware capture paths."""

from typing import Tuple


def center_crop_margins(width: int, height: int) -> Tuple[int, int, int, int]:
    """Return GStreamer left, right, top and bottom square-crop margins."""
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        raise ValueError('Camera width and height must be positive')

    horizontal = max(0, width - height)
    vertical = max(0, height - width)
    left = horizontal // 2
    right = horizontal - left
    top = vertical // 2
    bottom = vertical - top
    return left, right, top, bottom
