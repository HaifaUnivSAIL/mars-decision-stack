import math

from decision_agent.swarm_geometry import angle_delta, geodetic_to_local_xy, rotate_xy, world_to_body, wrap_angle


def test_wrap_angle_and_delta_are_normalized():
    assert math.isclose(wrap_angle(3.5), 3.5 - 2.0 * math.pi, rel_tol=0.0, abs_tol=1e-9)
    assert math.isclose(angle_delta(math.radians(-170.0), math.radians(170.0)), math.radians(20.0), abs_tol=1e-9)


def test_rotate_and_world_to_body_are_consistent():
    x_world, y_world = rotate_xy(1.0, 0.0, math.pi / 2.0)
    assert math.isclose(x_world, 0.0, abs_tol=1e-9)
    assert math.isclose(y_world, 1.0, abs_tol=1e-9)

    x_body, y_body = world_to_body(x_world, y_world, math.pi / 2.0)
    assert math.isclose(x_body, 1.0, abs_tol=1e-9)
    assert math.isclose(y_body, 0.0, abs_tol=1e-9)


def test_geodetic_to_local_xy_maps_small_offsets():
    x_east, y_north = geodetic_to_local_xy(32.0001, 34.0001, 32.0, 34.0)
    assert x_east > 0.0
    assert y_north > 0.0
    assert math.isclose(y_north, 11.1, rel_tol=0.05)
