"""Pure control-loop logic for the GPS-only autopilot.

No MQTT, no GPIO -- kept separate from autopilot.py so the steering math,
guardrails, and travel-budget bookkeeping can be unit-tested without a
broker or hardware. See GPS-ONLY-NAV.md for the design this implements.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def is_finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def wrap180(angle_deg):
    """Wraps to [-180, 180)."""
    return (angle_deg + 180) % 360 - 180


@dataclass
class GpsFix:
    cog_deg: float
    sog_kn: float
    received_at: float  # monotonic seconds, when this process received it


@dataclass
class NavIntent:
    route_bearing_deg: float
    cross_track_error_m: float
    received_at: float
    bearing_to_waypoint_deg: float = None


@dataclass
class Heartbeat:
    received_at: float


@dataclass
class GuardrailConfig:
    gps_max_age_s: float = 3.0
    heartbeat_max_age_s: float = 3.0
    route_max_age_s: float = 5.0
    # Back to 2 kn on 2026-08-04: steering held noticeably better in testing at
    # the higher gate. Nothing here filters COG (steering_error reads gps.cog_deg
    # raw), so below a couple of knots the autopilot chases COG noise -- the gate
    # is the only thing standing in for a filter. Revisit if COG ever gets smoothed.
    min_speed_kn: float = 2.0


def steer_enabled(now, gps, nav, heartbeat, engaged, config):
    """All of GPS-ONLY-NAV.md's guardrails, evaluated together, plus the explicit
    engage/disengage latch from OpenHelm.

    Returns (enabled, reasons); reasons lists every guardrail that failed,
    not just the first, so a log line (and the status topic) can show the
    full picture. `engaged` is a necessary but not sufficient condition:
    losing any other guardrail still disengages regardless of engage state.
    """
    reasons = []
    if not engaged:
        reasons.append("not engaged")
    if gps is None or now - gps.received_at > config.gps_max_age_s:
        reasons.append("gps fix stale")
    elif gps.sog_kn <= config.min_speed_kn:
        reasons.append("speed over ground too low")
    if heartbeat is None or now - heartbeat.received_at > config.heartbeat_max_age_s:
        reasons.append("openhelm heartbeat stale")
    if nav is None or now - nav.received_at > config.route_max_age_s:
        reasons.append("route data stale")
    return len(reasons) == 0, reasons


@dataclass
class SteeringConfig:
    # None of these are specified numerically in GPS-ONLY-NAV.md -- it leaves
    # "deadband" as a placeholder and says XTE "adjusts" the target track
    # without a formula. These are a starting point, tune on the water.
    deadband_deg: float = 5.0
    xte_correction_deg_per_meter: float = 3.0
    max_xte_correction_deg: float = 30.0
    # Position-based rudder law (2026-08-04 sea trial). The old law nudged the rudder
    # further over on every fix and never unwound it, so it wound up past half lock and
    # the boat could only keep turning -- a full 360 on the water. These command a rudder
    # *position* instead: proportional to error, damped by how fast she is already
    # swinging, and clamped well short of the stops.
    # 0.60: act on the course error properly. At 0.30 a 40deg error asked for 12s of
    # helm and sat there pinned while she drifted further off; this asks for 24s.
    rudder_s_per_error_deg: float = 0.60
    # Damping. Backed off from 1.8 after the second trial: turn rate comes from 1 Hz COG,
    # which jitters about +/-1 dps, and at 1.8 that noise alone commanded +/-1.8s of rudder
    # -- more helm than the real signal. The tracker is also smoothed harder now.
    rudder_s_per_turn_rate_dps: float = 1.2
    # Clamp opened to the full range on Evita's call, 2026-08-04: at 0.30 she sat pinned
    # at the limit 40deg off course, drifting further off, with nothing left to give.
    # The travel budget still stops the motor driving past hard_over_seconds, so this is
    # not unbounded -- but it does mean a reversed command can now reach full lock.
    max_rudder_fraction: float = 1.0
    max_pulse_ms: float = 300.0
    # Ceiling on auto-trim, as a fraction of the usable helm. Opened to the full range
    # on the 2026-08-04 trial: at 0.5 trim pinned at 20s while she still fell to port,
    # so the cap itself was what limited starboard helm. The demand clamp
    # (max_rudder_fraction) and the travel budget are the remaining backstops.
    max_trim_fraction: float = 1.0
    # Don't chase moves smaller than this. Raised from 0.08 after the second trial: the
    # demand wanders by a few tenths of a second on COG noise alone, and chasing every
    # wobble had the helm sawing +1/-1 at 300ms while the boat sat almost straight.
    rudder_deadband_s: float = 0.50


def target_track(nav, config):
    """Where to steer: the bearing from where we actually are to the next mark.

    Was "leg bearing, nudged by cross-track error, clamped to +/-30deg". That fails
    exactly when it matters: 600m off the line on the 2026-08-04 trial the correction
    sat saturated, so she was told to hold a heading 30deg off the leg and crab back to
    a line rather than simply go to the mark. Steering at the mark converges on it by
    construction and needs no clamp.

    Falls back to the old law when bearing_to_waypoint_deg is absent, so a NavIntent
    built without it (older callers, tests) still behaves as before.
    """
    if nav.bearing_to_waypoint_deg is not None:
        return wrap180(nav.bearing_to_waypoint_deg)
    correction = clamp(
        nav.cross_track_error_m * config.xte_correction_deg_per_meter,
        -config.max_xte_correction_deg,
        config.max_xte_correction_deg,
    )
    return wrap180(nav.route_bearing_deg - correction)


def steering_error(gps, nav, config, cog_deg=None):
    """cog_deg overrides the raw fix, so the loop can steer on smoothed course."""
    cog = gps.cog_deg if cog_deg is None else cog_deg
    return wrap180(target_track(nav, config) - cog)


def pulse_duration_ms(error_deg, deadband_deg):
    """The pulse-width table from GPS-ONLY-NAV.md's control loop. None = no pulse."""
    magnitude = abs(error_deg)
    if magnitude < deadband_deg:
        return None
    if magnitude < 10:
        return 100
    if magnitude < 20:
        return 200
    return 300


def command_sign_for_turn_right(direction_map):
    """Which command sign (+1/-1) turns the bow to starboard (increases COG).

    A rudder deflected to starboard turns the bow to starboard -- that's the
    standard rudder convention, not something calibrate.py measures directly.
    calibrate.py's direction_map only records which command sign deflects the
    rudder to which physical side; this combines the two facts.
    """
    positive_side = direction_map.get("positive_command")
    if positive_side == "starboard":
        return 1
    if positive_side == "port":
        return -1
    raise ValueError(f"unrecognized direction_map: {direction_map!r}")


@dataclass
class CogSmoother:
    """Low-pass on course over ground.

    Nothing filtered COG anywhere before this. The raw 1 Hz value wobbles a couple of
    degrees on sea state and GPS noise, and that went straight into both the heading
    error and the damping term -- on the 2026-08-04 trial a +/-2deg wobble swung the
    commanded rudder by about 1.4s while the boat was sitting nearly straight.

    Deliberately light. Smoothing trades noise for lag, and lag is what caused the
    oscillation this whole loop has been fighting; at 1 Hz an alpha of 0.4 settles in
    about two and a half seconds, which is short against how fast the boat answers.

    EMA over the *wrapped difference*, not the angle, so 359 -> 1 is a 2deg step
    rather than a 358deg one.
    """

    smoothing: float = 0.4
    cog_deg: float = None

    def update(self, raw_cog_deg):
        if self.cog_deg is None:
            self.cog_deg = raw_cog_deg % 360.0
        else:
            step = wrap180(raw_cog_deg - self.cog_deg)
            self.cog_deg = (self.cog_deg + self.smoothing * step) % 360.0
        return self.cog_deg


@dataclass
class TurnRateTracker:
    """Rate of course change, in deg/sec, positive when the bow swings to starboard.

    COG at 1 Hz is noisy, and an unsmoothed derivative would feed that noise straight
    into the damping term. The EMA is deliberately light so it doesn't lag the boat.
    """

    # Relaxed back to 0.5 now that CogSmoother cleans the input: double-filtering the
    # same noise buys little and costs lag, which is the more dangerous of the two.
    smoothing: float = 0.5
    # Two samples closer together than this can't be differentiated usefully. The loop
    # times fixes by when it *processed* them, not when they were taken, so a backlog
    # of 3 Hz T-Deck fixes draining after a blocking pulse can put two ticks a few ms
    # apart -- and a real COG step over a few ms reads as hundreds of deg/s. The
    # 2026-08-09 trial logged 918 deg/s that way; at Kd=1.2 the damping term then asked
    # for 1102 seconds of rudder the other way and pinned the helm wrong for ~5 s while
    # the EMA washed it out. See TRIAL-ANALYSIS-2026-08-09.md section 3.2.
    min_dt_s: float = 0.2
    # Backstop, not a model of the boat: she does not turn this fast, so anything
    # beyond it is a bad fix or a timing artefact rather than data worth steering on.
    max_rate_dps: float = 30.0
    rate_dps: float = 0.0
    _last_cog_deg: float = None
    _last_at_s: float = None

    def update(self, cog_deg, at_s):
        if self._last_cog_deg is None or self._last_at_s is None:
            self._last_cog_deg, self._last_at_s = cog_deg, at_s
            return self.rate_dps
        dt_s = at_s - self._last_at_s
        if dt_s < self.min_dt_s:
            # Hold the previous rate, and deliberately do NOT advance the reference:
            # the next fix then differentiates over a longer, usable interval measured
            # from the last sample we actually trusted.
            return self.rate_dps
        raw_dps = clamp(
            wrap180(cog_deg - self._last_cog_deg) / dt_s, -self.max_rate_dps, self.max_rate_dps
        )
        self.rate_dps = self.smoothing * raw_dps + (1.0 - self.smoothing) * self.rate_dps
        self._last_cog_deg, self._last_at_s = cog_deg, at_s
        return self.rate_dps


@dataclass
class HelmTrim:
    """Standing helm to carry a steady force -- weather helm, leeway, current.

    Proportional and damping terms alone cannot null a constant disturbance: the loop
    settles wherever Kp*error balances the force, so the error *has* to stay non-zero
    for the helm to stay on. Measured on the 2026-08-04 trial with a sail up: helm
    asked to port in 61% of samples averaging -0.98s, while the course error grew from
    7deg to 14deg and stayed there. This is the integral term autopilots call auto-trim.

    Slow on purpose. It exists to learn the wind's steady pull over tens of seconds;
    chasing gusts is the damping term's job.

    Three anti-windup guards, because an unguarded integrator on a boat is how you get
    a hard turn on exit from a manoeuvre:
      - the accumulator is clamped to max_trim_s
      - it does not integrate while the helm is already saturated -- otherwise trim
        keeps growing through a turn it cannot influence, then slams over on release
      - it does not integrate through a large error, which is a manoeuvre in progress,
        not a trim condition
    """

    # 0.006, not 0.003: at 0.003 a steady 20deg error builds only 0.06s of helm per
    # second, so trimming out a real sail load took minutes rather than tens of seconds.
    trim_s_per_deg_s: float = 0.006
    # 45deg, not 20deg. The guard is meant to stop trim winding up through a *manoeuvre*,
    # but on the 2026-08-04 trial the sail held her 20-25deg off -- straddling a 20deg
    # threshold, so trim froze at 0.23s and never cancelled the very force it exists for.
    # A genuine course change is far larger than this; a sail load is not.
    integrate_below_error_deg: float = 45.0
    trim_s: float = 0.0

    def update(self, error_deg, dt_s, max_trim_s, saturated=False):
        if dt_s <= 0 or saturated or abs(error_deg) > self.integrate_below_error_deg:
            return self.trim_s
        self.trim_s = clamp(
            self.trim_s + self.trim_s_per_deg_s * error_deg * dt_s, -max_trim_s, max_trim_s
        )
        return self.trim_s

    def reset(self):
        """Dropped whenever we stop steering, so a stale trim learned on one point of
        sail is never applied at the moment of re-engaging on another."""
        self.trim_s = 0.0


def target_rudder_s(error_deg, turn_rate_dps, max_rudder_s, direction_map, config, trim_s=0.0):
    """The rudder POSITION to hold, in signed seconds-of-travel (command-sign units).

    Proportional to the heading error, minus a term for the swing already underway --
    so a boat coming nicely onto course gets less helm, not more, and the rudder comes
    back toward centre by itself as the error closes.
    """
    turn_right_sign = command_sign_for_turn_right(direction_map)
    demand_s = turn_right_sign * (
        config.rudder_s_per_error_deg * error_deg
        - config.rudder_s_per_turn_rate_dps * turn_rate_dps
        + trim_s
    )
    return clamp(demand_s, -max_rudder_s, max_rudder_s)


def max_rudder_s_for(hard_over_seconds, config):
    smallest_hard_over_s = min(
        hard_over_seconds.get("positive", 0.0), hard_over_seconds.get("negative", 0.0)
    )
    return config.max_rudder_fraction * smallest_hard_over_s


def rudder_move(target_s, current_rudder_s, config):
    """Returns (command_sign, duration_ms) to slew the helm toward target_s, or None.

    This is the INNER loop and must run every tick, not once per GPS fix. Moving the
    rudder needs no new position data -- only deciding where to put it does. Gating this
    to 1 Hz capped the helm's slew rate at max_pulse_ms per second, so a demand of a few
    seconds of rudder took ten-plus seconds to reach; the boat had swung 60deg past by
    the time the helm arrived. That phase lag, not gain, drove the 2026-08-04 oscillation.
    """
    delta_s = target_s - current_rudder_s
    if abs(delta_s) < config.rudder_deadband_s:
        return None
    duration_ms = min(abs(delta_s) * 1000.0, config.max_pulse_ms)
    return (1 if delta_s > 0 else -1), duration_ms


def decide_rudder_pulse(error_deg, turn_rate_dps, current_rudder_s, hard_over_seconds,
                        direction_map, config):
    """Outer + inner in one call. Kept for tests and single-shot use; the control loop
    splits the two so the inner half can run at tick rate."""
    max_rudder_s = max_rudder_s_for(hard_over_seconds, config)
    target_s = target_rudder_s(error_deg, turn_rate_dps, max_rudder_s, direction_map, config)
    return rudder_move(target_s, current_rudder_s, config)


def decide_pulse(error_deg, direction_map, steering_config):
    """Returns (command_sign, duration_ms), or None if inside the deadband."""
    duration_ms = pulse_duration_ms(error_deg, steering_config.deadband_deg)
    if duration_ms is None:
        return None
    turn_right_sign = command_sign_for_turn_right(direction_map)
    sign = turn_right_sign if error_deg > 0 else -turn_right_sign
    return sign, duration_ms


def delivered_travel_s(duration_s, slew_per_second, max_duty):
    """Full-duty-equivalent seconds of rudder travel a pulse of duration_s really buys.

    The BTS7960 driver ramps duty from zero up to max_duty at slew_per_second, and
    drive_pulse disengages at the end of every pulse -- so every pulse restarts that
    ramp from nothing. At the vendored defaults (0.50/s toward a 0.353 cap) it takes
    0.71 s just to reach full commanded duty, which is longer than most pulses.

    Crediting the wall-clock pulse width as travel therefore over-reads badly, and the
    error is one-directional: it never under-reads. On the 2026-08-09 trial the budget
    counted 502 s of travel where this model says 279 s was delivered -- so the loop
    believed it held 38.8 s of helm when it held perhaps 21 s, hit rudder_move's
    deadband, and stopped pulsing with the rudder well short of the demand.

    Worst for short pulses: 100 ms only ever reaches duty 0.05, which will not shift a
    loaded helm at all, yet was credited a full 0.1 s.

    Returns the integral of duty over the pulse, normalised by max_duty, so the result
    is in the same units calibrate.py's hard_over_seconds is measured in (continuous
    drive at full commanded duty).
    """
    if duration_s <= 0 or slew_per_second <= 0 or max_duty <= 0:
        return 0.0
    ramp_s = min(duration_s, max_duty / slew_per_second)
    area = 0.5 * slew_per_second * ramp_s * ramp_s + max_duty * max(0.0, duration_s - ramp_s)
    return area / max_duty


@dataclass
class TravelBudget:
    """Tracks estimated rudder position as seconds-of-travel from center.

    GPS-ONLY-NAV.md: "cumulative motor-on time per direction, reset by known
    reference points (e.g. full-lock bench calibration)". hard_over_seconds
    comes straight from calibration.json and is the reset reference; there is
    no rudder position sensor, so this is an estimate that can drift and
    should be treated as advisory, not a guarantee against over-travel.
    """
    hard_over_seconds: dict
    position_s: float = 0.0

    def at_limit(self, sign):
        if sign > 0:
            return self.position_s >= self.hard_over_seconds.get("positive", 0.0)
        return self.position_s <= -self.hard_over_seconds.get("negative", 0.0)

    def remaining_s(self, sign):
        if sign > 0:
            return max(0.0, self.hard_over_seconds.get("positive", 0.0) - self.position_s)
        return max(0.0, self.position_s + self.hard_over_seconds.get("negative", 0.0))

    def allowed_pulse_s(self, sign, requested_s):
        return min(max(0.0, requested_s), self.remaining_s(sign))

    def apply_pulse(self, sign, duration_s):
        self.position_s += sign * duration_s
        self.position_s = clamp(
            self.position_s,
            -self.hard_over_seconds.get("negative", 0.0),
            self.hard_over_seconds.get("positive", 0.0),
        )


def load_calibration(path):
    path = Path(path)
    if not path.exists():
        raise ValueError(f"{path} not found -- run calibrate.py first")
    data = json.loads(path.read_text())
    direction_map = data.get("direction_map")
    hard_over_seconds = data.get("hard_over_seconds")
    if not isinstance(direction_map, dict) or not isinstance(hard_over_seconds, dict):
        raise ValueError(
            f"{path} is missing direction_map or hard_over_seconds -- run calibrate.py first"
        )

    positive_command = direction_map.get("positive_command")
    negative_command = direction_map.get("negative_command")
    if {positive_command, negative_command} != {"port", "starboard"}:
        raise ValueError(
            f"{path} direction_map must map positive/negative commands to opposite port/starboard sides"
        )

    for key in ("positive", "negative"):
        value = hard_over_seconds.get(key)
        if not is_finite_number(value) or value <= 0:
            raise ValueError(f"{path} hard_over_seconds.{key} must be a positive number")

    return direction_map, hard_over_seconds
