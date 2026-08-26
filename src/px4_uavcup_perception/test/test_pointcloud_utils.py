import numpy as np

from px4_uavcup_perception.depth.pointcloud import depth_to_flu_points


def test_depth_converts_from_optical_image_to_flu_points():
    depth = np.full((3, 3), 2.0, dtype=np.float32)

    points = depth_to_flu_points(
        depth,
        focal_x_px=2.0,
        focal_y_px=2.0,
        principal_x_px=1.0,
        principal_y_px=1.0,
        stride=1,
        minimum_depth_m=0.3,
        maximum_depth_m=8.0,
    )

    assert points.shape == (9, 3)
    np.testing.assert_allclose(points[4], [2.0, 0.0, 0.0])
    np.testing.assert_allclose(points[0], [2.0, 1.0, 1.0])
    np.testing.assert_allclose(points[8], [2.0, -1.0, -1.0])


def test_invalid_and_out_of_range_depth_are_removed():
    depth = np.array([
        [np.nan, 0.2],
        [2.0, 9.0],
    ], dtype=np.float32)

    points = depth_to_flu_points(
        depth,
        focal_x_px=2.0,
        focal_y_px=2.0,
        principal_x_px=1.0,
        principal_y_px=1.0,
        stride=1,
        minimum_depth_m=0.3,
        maximum_depth_m=8.0,
    )

    assert points.shape == (1, 3)
    np.testing.assert_allclose(points[0], [2.0, 1.0, 0.0])
