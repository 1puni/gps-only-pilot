"""Local route bearing / cross-track error, computed from GPS lat/lon alone.

OpenHelm doesn't publish bearing-to-waypoint or XTE over MQTT today (only
lat/lon/cog/sog/ts on the positions topic) -- passageNav.ts keeps that math
on-device for its own HUD. Rather than wait on an OpenHelm change, this
autopilot carries its own tiny waypoint list and does the same spherical
great-circle math itself, needing nothing from OpenHelm but position.
"""
import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6371000.0


def _rad(deg):
    return math.radians(deg)


def _deg(rad):
    return math.degrees(rad)


def initial_bearing_deg(lat1, lon1, lat2, lon2):
    phi1, phi2 = _rad(lat1), _rad(lat2)
    dlon = _rad(lon2 - lon1)
    y = math.sin(dlon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    return (_deg(math.atan2(y, x)) + 360) % 360


def angular_distance_rad(lat1, lon1, lat2, lon2):
    phi1, phi2 = _rad(lat1), _rad(lat2)
    dphi = _rad(lat2 - lat1)
    dlambda = _rad(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    a = clamp_unit(a)
    return 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def distance_m(lat1, lon1, lat2, lon2):
    return angular_distance_rad(lat1, lon1, lat2, lon2) * EARTH_RADIUS_M


def cross_track_error_m(fix_lat, fix_lon, start, end):
    """Signed distance of the fix from the great-circle path start->end.

    Positive = fix is to the right of the path, negative = left (the
    Movable Type / aviation-formulary convention). Good enough at
    coastal/archipelago distances; doesn't need to be geodesically exact.
    """
    start_lat, start_lon = start
    end_lat, end_lon = end
    d13 = angular_distance_rad(start_lat, start_lon, fix_lat, fix_lon)
    brng13 = _rad(initial_bearing_deg(start_lat, start_lon, fix_lat, fix_lon))
    brng12 = _rad(initial_bearing_deg(start_lat, start_lon, end_lat, end_lon))
    xte_rad = math.asin(clamp_unit(math.sin(d13) * math.sin(brng13 - brng12)))
    return xte_rad * EARTH_RADIUS_M


def along_track_m(fix_lat, fix_lon, start, end):
    """Distance along start->end to the foot of the perpendicular from the fix.

    Negative when the fix lies behind `start`. Paired with cross_track_error_m this
    says *where along the leg* the vessel is; a value >= the leg's length means it has
    crossed the perpendicular at `end` -- i.e. passed that mark abeam. Mirrors
    OpenHelm's passageNav.alongTrackNm so both ends agree on when a mark is behind us.
    """
    start_lat, start_lon = start
    end_lat, end_lon = end
    d13 = angular_distance_rad(start_lat, start_lon, fix_lat, fix_lon)
    brng13 = _rad(initial_bearing_deg(start_lat, start_lon, fix_lat, fix_lon))
    brng12 = _rad(initial_bearing_deg(start_lat, start_lon, end_lat, end_lon))
    dxt = math.asin(clamp_unit(math.sin(d13) * math.sin(brng13 - brng12)))
    dat = math.acos(clamp_unit(math.cos(d13) / math.cos(dxt))) * EARTH_RADIUS_M
    return -dat if math.cos(brng13 - brng12) < 0 else dat


def clamp_unit(value):
    """asin's domain is [-1, 1]; float error can push sin*sin just outside it."""
    return min(max(value, -1.0), 1.0)


def validate_leg_index(leg_index, waypoint_count):
    """A leg is identified by its starting waypoint, so the last valid leg on an
    n-waypoint route is n-2 (the segment into the final mark)."""
    if not isinstance(leg_index, int) or isinstance(leg_index, bool):
        raise ValueError(f"invalid leg_index: {leg_index!r}")
    last_leg = waypoint_count - 2
    if not 0 <= leg_index <= last_leg:
        raise ValueError(f"leg_index {leg_index} out of range 0..{last_leg}")
    return leg_index


def validate_waypoint(waypoint):
    if not isinstance(waypoint, (list, tuple)) or len(waypoint) != 2:
        raise ValueError(f"invalid waypoint: {waypoint!r}")
    lat, lon = waypoint
    if not is_finite_number(lat) or not -90 <= lat <= 90:
        raise ValueError(f"invalid waypoint latitude: {waypoint!r}")
    if not is_finite_number(lon) or not -180 <= lon <= 180:
        raise ValueError(f"invalid waypoint longitude: {waypoint!r}")
    return float(lat), float(lon)


def is_finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass
class NavFix:
    route_bearing_deg: float  # bearing of the leg itself, start -> end
    bearing_to_waypoint_deg: float  # bearing from where we actually are to the mark
    cross_track_error_m: float
    distance_to_waypoint_m: float
    leg_index: int
    arrived: bool


class Route:
    """An ordered list of (lat, lon) waypoints, tracking the active leg."""

    def __init__(self, waypoints, arrival_radius_m=30.0, leg_index=0):
        if not isinstance(waypoints, (list, tuple)):
            raise ValueError("route waypoints must be a list")
        if len(waypoints) < 2:
            raise ValueError("route needs at least 2 waypoints (start + destination)")
        if not is_finite_number(arrival_radius_m) or arrival_radius_m <= 0:
            raise ValueError("arrival_radius_m must be a positive number")
        self.waypoints = [validate_waypoint(waypoint) for waypoint in waypoints]
        self.arrival_radius_m = arrival_radius_m
        self.leg_index = validate_leg_index(leg_index, len(self.waypoints))

    def set_leg_index(self, leg_index):
        """Point the route at a chosen leg -- OpenHelm's "skip"/"set as next mark", i.e.
        the operator's authority when our own advance guessed wrong.

        An absolute set, so it may move backward. Note that update() still refines
        forward from here, so a mark already passed abeam stays passed -- the same
        behaviour as OpenHelm's setActiveLeg, deliberately, so the two ends agree.
        """
        self.leg_index = validate_leg_index(leg_index, len(self.waypoints))

    @property
    def final_leg(self):
        return self.leg_index >= len(self.waypoints) - 2

    def _mark_is_behind(self, fix_lat, fix_lon, index):
        """Is the mark at the end of leg `index` behind us?

        True when inside the arrival radius, OR when we've crossed the perpendicular at
        the mark. The radius alone strands the route when a mark is rounded wide, when
        the boat joins mid-leg, or when the route is edited: miss by more than the
        radius and the leg never advances, leaving the autopilot steering at a mark it
        already sailed past. Forward-only, so GPS noise can't walk the leg backward.
        """
        start, end = self._leg(index)
        if distance_m(fix_lat, fix_lon, *end) <= self.arrival_radius_m:
            return True
        leg_length_m = distance_m(*start, *end)
        if leg_length_m <= 1e-6:  # degenerate leg has no direction to be abeam of
            return False
        return along_track_m(fix_lat, fix_lon, start, end) >= leg_length_m

    def update(self, fix_lat, fix_lon):
        """Advances past every mark already behind us, then reports bearing/XTE for
        whichever leg is now active."""
        while not self.final_leg and self._mark_is_behind(fix_lat, fix_lon, self.leg_index):
            self.leg_index += 1

        start, end = self._leg(self.leg_index)
        distance_to_go = distance_m(fix_lat, fix_lon, *end)
        arrived = self.final_leg and self._mark_is_behind(fix_lat, fix_lon, self.leg_index)
        return NavFix(
            route_bearing_deg=initial_bearing_deg(*start, *end),
            bearing_to_waypoint_deg=initial_bearing_deg(fix_lat, fix_lon, *end),
            cross_track_error_m=cross_track_error_m(fix_lat, fix_lon, start, end),
            distance_to_waypoint_m=distance_to_go,
            leg_index=self.leg_index,
            arrived=arrived,
        )

    def _leg(self, index):
        return self.waypoints[index], self.waypoints[index + 1]
