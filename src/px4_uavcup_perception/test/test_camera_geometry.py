from px4_uavcup_perception.common.camera_geometry import center_crop_margins


def test_landscape_frame_is_cropped_equally_from_left_and_right():
    assert center_crop_margins(1280, 720) == (280, 280, 0, 0)


def test_portrait_frame_is_cropped_equally_from_top_and_bottom():
    assert center_crop_margins(720, 1280) == (0, 0, 280, 280)


def test_odd_difference_preserves_exact_square_size():
    left, right, top, bottom = center_crop_margins(1281, 720)

    assert (left, right, top, bottom) == (280, 281, 0, 0)
    assert 1281 - left - right == 720
