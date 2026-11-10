# Gamepad control of the Gazebo follow camera (sim/camera_orbit.py), without a gamepad or Gazebo.
import math

import numpy

from camera_orbit import EVENT, JS_EVENT_AXIS, JS_EVENT_BUTTON, JS_EVENT_INIT, Orbit, Pad, pressed, to_bgr

RELEASED, FULL = -32767, 32767


def event(kind, number, value):
    return EVENT.pack(0, value, kind, number)


def test_starts_where_px4_puts_the_follow_camera():
    # px4-rc.gzsim sets the follow offset to (-2, -2, 2).
    assert Orbit().offset() == (-2.0, -2.0, 2.0)


def test_dpad_left_right_orbits_at_constant_distance_and_height():
    orbit = Orbit()
    orbit.step(hat_x=1, hat_y=0, zoom_in=0, zoom_out=0, dt=1.5)  # 90 deg
    assert orbit.offset() == (-2.0, 2.0, 2.0)
    for _ in range(3):
        orbit.step(hat_x=-1, hat_y=0, zoom_in=0, zoom_out=0, dt=2.0)  # a full turn the other way
    assert orbit.offset() == (-2.0, 2.0, 2.0)
    orbit.step(hat_x=-1, hat_y=0, zoom_in=0, zoom_out=0, dt=1.5)  # and 90 deg back to the start
    assert orbit.offset() == (-2.0, -2.0, 2.0)
    assert -180.0 <= orbit.azimuth < 180.0


def test_dpad_up_raises_down_lowers_the_camera_within_limits():
    orbit = Orbit()
    orbit.step(0, -1, 0, 0, dt=1.0)  # up
    assert orbit.height == 3.0
    orbit.step(0, 1, 0, 0, dt=100.0)
    assert orbit.height == -1.0
    orbit.step(0, -1, 0, 0, dt=100.0)
    assert orbit.height == 10.0


def test_rt_zooms_in_lt_zooms_out_harder_is_faster_within_limits():
    soft, hard = Orbit(), Orbit()
    start = soft.distance
    soft.step(0, 0, zoom_in=0.3, zoom_out=0, dt=0.5)
    hard.step(0, 0, zoom_in=1.0, zoom_out=0, dt=0.5)
    assert hard.distance < soft.distance < start
    hard.step(0, 0, zoom_in=0, zoom_out=1.0, dt=100.0)
    assert hard.distance == 15.0
    hard.step(0, 0, zoom_in=1.0, zoom_out=0, dt=100.0)
    assert hard.distance == 1.0
    # Zoom keeps the direction to the drone.
    x, y, _ = hard.offset()
    assert math.isclose(math.degrees(math.atan2(y, x)), -135.0)


def test_trigger_travel_maps_to_zero_to_one_with_a_small_dead_zone():
    assert pressed(RELEASED) == 0.0
    assert pressed(-30000) == 0.0  # resting slightly off released
    assert pressed(FULL) == 1.0
    assert 0.4 < pressed(0) < 0.6


def test_pad_reads_dpad_and_triggers_and_ignores_the_flying_sticks():
    pad = Pad()
    assert not pad.moving()  # triggers count as released before the pad reports them
    for number in (0, 1, 3, 4):  # sticks: QGroundControl flies with these
        pad.apply(event(JS_EVENT_AXIS, number, FULL))
    for number in (2, 5):  # state of the triggers sent when the device is opened
        pad.apply(event(JS_EVENT_AXIS | JS_EVENT_INIT, number, RELEASED))
    assert not pad.moving()
    pad.apply(event(JS_EVENT_AXIS, 6, -32767))
    pad.apply(event(JS_EVENT_AXIS, 7, 32767))
    assert (pad.hat_x, pad.hat_y) == (-1, 1) and pad.moving()
    pad.apply(event(JS_EVENT_AXIS, 6, 0))
    pad.apply(event(JS_EVENT_AXIS, 7, 0))
    assert not pad.moving()
    pad.apply(event(JS_EVENT_AXIS, 5, FULL))  # RT
    assert (pad.zoom_in, pad.zoom_out) == (1.0, 0.0) and pad.moving()
    pad.apply(event(JS_EVENT_AXIS, 5, RELEASED))
    pad.apply(event(JS_EVENT_AXIS, 2, FULL))  # LT
    assert (pad.zoom_in, pad.zoom_out) == (0.0, 1.0)


def test_buttons_report_each_new_press_once_and_initial_state_is_not_a_press():
    pad = Pad()
    assert pad.apply(event(JS_EVENT_BUTTON | JS_EVENT_INIT, 3, 1)) is None  # held when the pad was opened
    assert pad.apply(event(JS_EVENT_BUTTON, 3, 0)) is None
    assert pad.apply(event(JS_EVENT_BUTTON, 3, 1)) == 3  # Y
    assert pad.apply(event(JS_EVENT_BUTTON, 3, 1)) is None  # repeat while held
    assert pad.apply(event(JS_EVENT_BUTTON, 3, 0)) is None
    assert pad.apply(event(JS_EVENT_BUTTON, 2, 1)) == 2  # X
    assert pad.apply(event(JS_EVENT_AXIS, 6, FULL)) is None


def test_gazebo_rgb_image_becomes_opencv_bgr():
    # 2x1 image: a red pixel then a blue one, as Gazebo sends RGB_INT8.
    bgr = to_bgr(2, 1, bytes([255, 0, 0, 0, 0, 255]))
    assert bgr.shape == (1, 2, 3)
    assert numpy.array_equal(bgr[0, 0], [0, 0, 255]) and numpy.array_equal(bgr[0, 1], [255, 0, 0])
