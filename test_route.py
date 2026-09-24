import math

import pytest

from route import (
    Route,
    along_track_m,
    cross_track_error_m,
    distance_m,
    initial_bearing_deg,
)


def test_initial_bearing_due_north():
    assert initial_bearing_deg(0, 0, 1, 0) == pytest.approx(0, abs=1e-6)


def test_initial_bearing_due_east():
    assert initial_bearing_deg(0, 0, 0, 1) == pytest.approx(90, abs=1e-6)


def test_distance_m_one_degree_latitude_is_about_111km():
    assert distance_m(0, 0, 1, 0) == pytest.approx(111195, rel=1e-3)


def test_xte_positive_when_fix_is_right_of_eastward_track():
    # Track heads due east (bearing 90); a fix south of it is to the right.
    xte = cross_track_error_m(-0.01, 0.5, (0, 0), (0, 1))
    assert xte > 0


def test_xte_negative_when_fix_is_left_of_eastward_track():
    # Same track; a fix north of it is to the left.
    xte = cross_track_error_m(0.01, 0.5, (0, 0), (0, 1))
    assert xte < 0


def test_xte_near_zero_on_track():
    xte = cross_track_error_m(0.0, 0.5, (0, 0), (0, 1))
    assert xte == pytest.approx(0, abs=1.0)


def test_route_rejects_single_waypoint():
    with pytest.raises(ValueError):
        Route([(0, 0)])


def test_route_rejects_non_list_waypoints():
    with pytest.raises(ValueError):
        Route(None)


def test_route_rejects_invalid_waypoint_coordinates():
    with pytest.raises(ValueError):
        Route([(0, 0), (91, 1)])


def test_route_rejects_nonpositive_arrival_radius():
    with pytest.raises(ValueError):
        Route([(0, 0), (0, 1)], arrival_radius_m=0)


def test_route_reports_first_leg_bearing_and_xte():
    route = Route([(0, 0), (0, 1), (1, 1)], arrival_radius_m=30)
    nav = route.update(0.0, 0.5)
    assert nav.leg_index == 0
    assert nav.route_bearing_deg == pytest.approx(90, abs=1e-6)
    assert nav.arrived is False


def test_route_advances_leg_on_arrival():
    route = Route([(0, 0), (0, 1), (1, 1)], arrival_radius_m=30)
    # Close enough to the first waypoint's destination (0, 1) to advance.
    nav = route.update(0.0, 1.0)
    assert nav.leg_index == 1
    assert nav.route_bearing_deg == pytest.approx(0, abs=1e-6)  # due north, leg 2


def test_route_reports_arrived_at_final_waypoint():
    route = Route([(0, 0), (0, 1)], arrival_radius_m=30)
    nav = route.update(0.0, 1.0)
    assert nav.arrived is True


def test_route_does_not_advance_past_final_leg():
    route = Route([(0, 0), (0, 1)], arrival_radius_m=30)
    route.update(0.0, 1.0)
    assert route.final_leg is True
    nav = route.update(0.0, 1.0)
    assert nav.leg_index == 0


def test_along_track_is_half_the_leg_at_the_midpoint():
    leg_m = distance_m(0, 0, 0, 1)
    assert along_track_m(0.0, 0.5, (0, 0), (0, 1)) == pytest.approx(leg_m / 2, rel=1e-3)


def test_along_track_negative_when_fix_lies_behind_the_start():
    assert along_track_m(0.0, -0.5, (0, 0), (0, 1)) < 0


# --- passing a mark abeam -------------------------------------------------------------
# The arrival radius alone strands the route when a mark is rounded wide: miss by more
# than the radius and the leg never advances, leaving the autopilot steering at a mark
# already astern.

def test_route_advances_past_a_mark_rounded_wide():
    route = Route([(0, 0), (0, 1), (1, 1)], arrival_radius_m=30)
    # Well past the perpendicular at (0, 1), and ~5.6 km off it -- far outside the radius.
    nav = route.update(0.01, 1.05)
    assert nav.leg_index == 1


def test_route_arrives_when_final_mark_is_passed_wide():
    route = Route([(0, 0), (0, 1)], arrival_radius_m=30)
    nav = route.update(0.01, 1.05)
    assert nav.arrived is True


def test_route_does_not_advance_before_the_mark():
    route = Route([(0, 0), (0, 1), (1, 1)], arrival_radius_m=30)
    nav = route.update(0.01, 0.95)  # short of the perpendicular, off to one side
    assert nav.leg_index == 0


# --- leg_index pushed by OpenHelm -----------------------------------------------------

def test_route_defaults_to_leg_zero():
    assert Route([(0, 0), (0, 1), (1, 1)]).leg_index == 0


def test_route_honours_pushed_leg_index():
    route = Route([(0, 0), (0, 1), (1, 1)], leg_index=1)
    assert route.leg_index == 1


def test_route_rejects_leg_index_past_the_final_leg():
    with pytest.raises(ValueError):
        Route([(0, 0), (0, 1), (1, 1)], leg_index=2)  # only legs 0 and 1 exist


def test_route_rejects_negative_leg_index():
    with pytest.raises(ValueError):
        Route([(0, 0), (0, 1)], leg_index=-1)


def test_route_rejects_non_integer_leg_index():
    with pytest.raises(ValueError):
        Route([(0, 0), (0, 1)], leg_index=0.5)


def test_route_rejects_boolean_leg_index():
    # bool is an int subclass; True must not silently mean leg 1.
    with pytest.raises(ValueError):
        Route([(0, 0), (0, 1), (1, 1)], leg_index=True)


def test_set_leg_index_moves_the_active_leg():
    route = Route([(0, 0), (0, 1), (1, 1)])
    route.set_leg_index(1)
    assert route.leg_index == 1


def test_set_leg_index_rejects_out_of_range():
    route = Route([(0, 0), (0, 1), (1, 1)])
    with pytest.raises(ValueError):
        route.set_leg_index(5)
    assert route.leg_index == 0  # unchanged


def test_pushed_leg_index_still_refines_forward():
    # Seeded behind where the boat actually is; update() must catch up rather than
    # steer at a mark already astern (mirrors OpenHelm's setActiveLeg semantics).
    route = Route([(0, 0), (0, 1), (1, 1)], arrival_radius_m=30, leg_index=0)
    nav = route.update(0.01, 1.05)
    assert nav.leg_index == 1


def test_nav_fix_reports_bearing_from_the_boat_to_the_mark():
    # Leg runs due east along the equator; the boat sits well south of it, so the
    # bearing to the mark is north-east-ish even though the leg bearing is 090.
    route = Route([(0, 0), (0, 1)], arrival_radius_m=30)
    nav = route.update(-0.5, 0.5)
    assert nav.route_bearing_deg == pytest.approx(90, abs=1e-6)
    assert 0 < nav.bearing_to_waypoint_deg < 90
