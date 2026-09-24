#!/usr/bin/env python3
"""GPS-only autopilot control loop. See GPS-ONLY-NAV.md for the design.

Reads phone GPS (lat/lon/cog/sog) from OpenHelm on the dedicated
`openhelm/autopilot/<vesselId>/gps` topic, or from the optional direct HTTP
test server. Auto steering only runs while OpenHelm has published a retained
engage command and the existing guardrails pass. Manual jog commands are
momentary hold-refresh commands and bypass route-following, while still
respecting the travel budget from calibration.

OpenHelm does not publish route bearing or cross-track error, so this script
computes both itself (route.py) from a route pushed by OpenHelm's route editor
or, before the first push, a local bootstrap route file if configured.

Drives the BTS7960 motor directly via the vendored driver, bypassing
pypilot's nav/heading logic, per GPS-ONLY-NAV.md's "not pypilot" v1 split.

The direct HTTP server is for dockside/Mac-to-engine testing without broker
credentials. Enable it with AUTOPILOT_DIRECT_HTTP_PORT=8765 and set that URL
in OpenHelm's Autopilot tab.
"""
import json
import os
import socket
import ssl
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib import request

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

import control
import learn
import nmea
import route as route_module
from vendor.bts7960_servo import BTS7960Config, BTS7960ServoDriver

AUTOPILOT_PREFIX = "openhelm/autopilot/"
VESSEL_ID = os.environ.get("AUTOPILOT_VESSEL_ID", "vessel")
GPS_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/gps"
ENGAGE_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/engage"
ROUTE_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/route"
LEG_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/leg"
MANUAL_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/manual"
PULSE_MODE_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/pulse_mode"
STATUS_TOPIC = f"{AUTOPILOT_PREFIX}{VESSEL_ID}/status"
CONTROL_TICK_S = 0.1
DRIVE_REFRESH_S = 0.1  # must stay well under BTS7960Config.watchdog_seconds
MANUAL_PULSE_S = 0.1
MANUAL_MAX_AGE_S = 0.7
UNUSABLE_FIX_LOG_INTERVAL_S = 10.0
NO_CALIBRATION_REASON = "calibration missing"

# GPS sources. The T-Deck broadcasts its u-blox M10's own NMEA over UDP; the
# phone pushes JSON over MQTT/HTTP. The T-Deck is preferred, the phone is the
# fallback -- see LiveNav._should_accept.
SOURCE_TDECK = "tdeck"
SOURCE_PHONE = "phone"
# How long the T-Deck must be silent before the phone is listened to again.
# Comfortably longer than a couple of missed datagrams at 3 Hz, short enough that
# a dead T-Deck doesn't cost a whole leg.
SOURCE_FALLBACK_S = 5.0
DEFAULT_NMEA_UDP_PORT = 10110

# learn.py wiring -- see docs/superpowers/specs/2026-07-23-adaptive-boat-response-model-design.md
DEFAULT_OBSERVE_FIXES = 3
RESPONSE_LEARNING_RATE = 0.000002
# Pulses to watch before seeding the response model from observed COG, when
# calibrate.py's rudder-angle table is absent. Small enough to seed within a few
# minutes of steering, large enough for the median to reject one bad fix.
DEFAULT_SEED_SAMPLES = 5
# Hard ceiling on a learned pulse. predict_pulse_ms() is desired_rate / response_rate,
# which is unbounded as the rate goes small -- a badly seeded model could otherwise ask
# for a multi-second slam of the helm, and the travel budget would happily grant it if
# the rudder had the room. Deliberately well above the fixed table's 300ms, since the
# learned model is *meant* to outrun it; this only clips pathological values.
MAX_LEARNED_PULSE_MS = 1000.0
OUTLIER_HISTORY_LEN = 20


def pulse_mode_from_env():
    mode = os.environ.get("PULSE_MODE", "fixed")
    if mode not in ("fixed", "learned"):
        raise ValueError(f"PULSE_MODE must be 'fixed' or 'learned', got {mode!r}")
    return mode


class LiveNav:
    """Thread-safe holder for the latest OpenHelm GPS fix."""

    def __init__(self):
        self._lock = threading.Lock()
        self._new_fix = threading.Event()
        self.gps = None
        self.heartbeat = None
        self.lat = None
        self.lon = None
        # The reason the most recent payload was unusable, or None if it was fine.
        # A plain attribute assignment is atomic under the GIL, same reasoning as
        # EngageState, so the control loop reads it without taking the lock.
        self.gps_problem = None
        self._last_problem = None
        self._last_problem_log_at = None
        # Which source we would rather steer on, and when it last spoke. See
        # _should_accept for why the fallback is one-way.
        self.preferred_source = SOURCE_TDECK
        self.active_source = None
        self._preferred_seen_at = None
        # The most recent position we have been able to decode, regardless of
        # whether it was usable for steering. COG is empty when the boat is
        # stationary, and that used to discard the whole fix -- leaving the
        # status endpoint (and a phone sourcing GPS from the T-Deck) with no
        # idea where the boat was at the dock. The position is still real, so we
        # keep it here for status consumers; the guardrails still refuse to steer
        # without a course. (lat, lon, cog, sog, received_at_monotonic).
        self.last_position = None

    def _log_problem(self, problem, now):
        """Logs an unusable payload, rate-limited: at 1 Hz an unset cog/sog would
        otherwise write one line per second for the whole passage. Re-logs
        immediately when the reason changes, so a switch of failing field shows up."""
        if problem == self._last_problem and self._last_problem_log_at is not None:
            if now - self._last_problem_log_at < UNUSABLE_FIX_LOG_INTERVAL_S:
                return
        self._last_problem = problem
        self._last_problem_log_at = now
        print(f"autopilot: unusable gps payload -- {problem}")

    def _should_accept(self, source, now):
        """Decides whether a fix from `source` wins.

        The T-Deck is the GPS of record: it feeds a u-blox M10 at nav rate, where
        the phone gives us whatever Android felt like reporting. But we keep the
        phone wired up so a T-Deck failure at sea degrades instead of stopping.

        The rule is deliberately one-way: the preferred source always wins, and
        the fallback is only heard once the preferred one has gone quiet for
        SOURCE_FALLBACK_S. Without that, two live sources at different rates
        would interleave and the control loop would see COG jitter that is really
        just alternating receivers.
        """
        if source == self.preferred_source:
            return True
        if self._preferred_seen_at is None:
            return True  # preferred source has never reported; nothing to defer to
        return now - self._preferred_seen_at >= SOURCE_FALLBACK_S

    def apply_fix(self, fix, problem, source, now=None):
        """Records a decoded fix. `fix` is (lat, lon, cog, sog) or None.

        Both the MQTT/HTTP payload path and the UDP NMEA path land here, so the
        staleness and arbitration rules live in exactly one place.
        """
        if now is None:
            now = time.monotonic()

        with self._lock:
            if not self._should_accept(source, now):
                return False
            if source == self.preferred_source:
                self._preferred_seen_at = now

            if source != self.active_source:
                print(f"autopilot: gps source -> {source}")
                self.active_source = source

            self.heartbeat = control.Heartbeat(received_at=now)
            self.gps_problem = problem
            if fix is None:
                # The source is alive (heartbeat above still updated) but has no
                # usable fix yet -- don't steer on it.
                self._log_problem(problem, now)
                self.gps = None
                self.lat = None
                self.lon = None
            else:
                lat, lon, cog, sog = fix
                self.gps = control.GpsFix(cog_deg=cog, sog_kn=sog, received_at=now)
                self.lat, self.lon = lat, lon
                self.last_position = (lat, lon, cog, sog, now)
        self._new_fix.set()
        return True

    def record_position(self, lat, lon, cog, sog, now=None):
        """Records a position that carries no usable steering fix (empty COG at
        the dock) so status consumers still know where the boat is. Does not
        touch the steering-grade fix or the source arbitration."""
        if now is None:
            now = time.monotonic()
        with self._lock:
            self.last_position = (lat, lon, cog, sog, now)

    def apply(self, data, source=SOURCE_PHONE, now=None):
        """Applies a decoded OpenHelm JSON payload."""
        fix, problem = inspect_position_payload(data)
        return self.apply_fix(fix, problem, source, now)

    def on_message(self, _client, _userdata, msg):
        """paho-mqtt adapter: unwrap the transport, then hand over the payload."""
        if getattr(msg, "retain", False):
            return
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        self.apply(data, SOURCE_PHONE)

    def wait_for_new_fix(self, timeout):
        got_one = self._new_fix.wait(timeout)
        self._new_fix.clear()
        return got_one

    def snapshot(self):
        with self._lock:
            return self.gps, self.heartbeat, self.lat, self.lon


class EngageState:
    """Thread-safe holder for the latest retained engage/disengage command.

    One-shot latch, not a refreshed heartbeat (see the design spec): OpenHelm
    publishes retained {"engaged": bool} on press; this holds it until the
    next message. A single bool attribute is already atomic under the GIL,
    so reads elsewhere in this module don't need a lock -- only the parse
    step needs to guard against a malformed payload leaving it unset.
    """

    def __init__(self):
        self.engaged = False

    def on_message(self, _client, _userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        engaged = data.get("engaged")
        if not isinstance(engaged, bool):
            return
        self.engaged = engaged


class PulseModeState:
    """Thread-safe holder for the live-switchable PULSE_MODE.

    Retained latch, same shape as EngageState: OpenHelm publishes retained
    {"mode": "fixed"|"learned"} and this holds it until the next message, so
    the driving path can be switched on the fly without restarting the
    process. Seeded from the PULSE_MODE env var (pulse_mode_from_env) so a
    session starts wherever the operator last set it.
    """

    def __init__(self, initial_mode="fixed"):
        self.mode = initial_mode

    def on_message(self, _client, _userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        mode = data.get("mode")
        if mode not in ("fixed", "learned"):
            return
        self.mode = mode


class RouteState:
    """Thread-safe holder for the current route, replaceable by a pushed route message.

    Falls back to whatever `initial_route` was loaded from AUTOPILOT_ROUTE_FILE
    until the first `route` message ever arrives; retained pushes then win on
    every later push, reconnect, and process restart (the design spec's
    "full replace, always restart at leg 0" -- Route.__init__ already starts
    leg_index at 0, and Route.update()'s while-loop fast-forwards past any
    already-passed waypoints on the very next fix). A malformed push is
    rejected (logged, previous route kept) rather than crashing the process.
    """

    def __init__(self, initial_route=None):
        self.current = initial_route

    def on_message(self, _client, _userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            print("autopilot: rejected pushed route -- payload is not valid JSON")
            return False
        if not isinstance(data, dict):
            print("autopilot: rejected pushed route -- payload is not an object")
            return False
        waypoints = data.get("waypoints")
        if waypoints is None:
            print("autopilot: rejected pushed route -- payload has no waypoints")
            return False
        arrival_radius_m = data.get("arrival_radius_m", 30.0)
        # Absent leg_index means an OpenHelm build that doesn't send one (or no passage
        # under way): start at leg 0, exactly as before.
        leg_index = data.get("leg_index", 0)
        try:
            new_route = route_module.Route(
                waypoints, arrival_radius_m=arrival_radius_m, leg_index=leg_index
            )
        except ValueError as e:
            print(f"autopilot: rejected pushed route -- {e}")
            return False
        self.current = new_route
        print(
            f"autopilot: route replaced ({len(new_route.waypoints)} waypoints, "
            f"starting leg {new_route.leg_index})"
        )
        return True

    def on_leg_message(self, _client, _userdata, msg):
        """OpenHelm moved its active leg (skip / "set as next mark") -- follow it, so the
        phone's next mark and ours can't disagree about where we're steering."""
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            print("autopilot: rejected leg -- payload is not valid JSON")
            return False
        if not isinstance(data, dict):
            print("autopilot: rejected leg -- payload is not an object")
            return False
        route = self.current
        if route is None:
            print("autopilot: rejected leg -- no route loaded")
            return False
        try:
            route.set_leg_index(data.get("leg_index"))
        except ValueError as e:
            print(f"autopilot: rejected leg -- {e}")
            return False
        print(f"autopilot: active leg set to {route.leg_index}")
        return True


class ManualState:
    """Momentary manual-jog state.

    OpenHelm sends a direction repeatedly while the user is holding a button,
    and sends null on release. If the stream goes quiet, this expires quickly.
    """

    def __init__(self, max_age_s=MANUAL_MAX_AGE_S):
        self.max_age_s = max_age_s
        self._lock = threading.Lock()
        self.direction = None
        self.received_at = 0.0

    def on_message(self, _client, _userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        direction = data.get("direction")
        if direction not in ("port", "starboard", None):
            return
        with self._lock:
            self.direction = direction
            self.received_at = time.monotonic()

    def current(self, now):
        with self._lock:
            if self.direction is None:
                return None
            if now - self.received_at > self.max_age_s:
                self.direction = None
                return None
            return self.direction


class StatusState:
    def __init__(self):
        self._lock = threading.Lock()
        self.current = None

    def set(self, status):
        with self._lock:
            self.current = dict(status)

    def snapshot(self):
        with self._lock:
            return dict(self.current) if self.current is not None else {}


class MotorWebManualProxy:
    """Manual-only bridge to the already-running dockside motor web service.

    This is intentionally only used when calibration is missing. It keeps one
    process owning the GPIO pins (`motor_web.py`) while exposing the CORS-clean
    OpenHelm `/autopilot/publish` endpoint on port 8765.
    """

    def __init__(self, base_url, duty=90.0, starboard_sign=1):
        self.base_url = base_url.rstrip("/")
        self.duty = float(duty)
        self.starboard_sign = 1 if int(starboard_sign) >= 0 else -1

    def _post(self, path, payload):
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(
            f"{self.base_url}{path}",
            data=data,
            headers={"content-type": "application/json"},
            method="POST",
        )
        with request.urlopen(req, timeout=0.75) as resp:
            body = resp.read()
            return json.loads(body.decode("utf-8")) if body else {}

    def apply(self, direction):
        if direction is None:
            return self._post("/api/stop", {})
        if direction not in ("port", "starboard"):
            raise ValueError(f"invalid manual direction: {direction!r}")
        sign = self.starboard_sign if direction == "starboard" else -self.starboard_sign
        return self._post("/api/drive", {"direction": sign, "duty": self.duty})


def inspect_position_payload(data):
    """Returns (fix, problem): the (lat, lon, cog, sog) tuple for a usable fix, or
    None plus a short description of the first field that made it unusable.

    OpenHelm forwards whatever the phone's platform gives it, and Android leaves
    cog/sog unset often enough that "gps fix stale" with a healthy heartbeat is a
    normal-looking symptom of a perfectly healthy link. Naming the field is the
    difference between diagnosing that in a minute and guessing at it.
    """
    values = {}
    for key in ("lat", "lon", "cog", "sog"):
        value = data.get(key)
        if not control.is_finite_number(value):
            return None, f"{key} is {value!r}"
        values[key] = float(value)

    lat, lon, cog, sog = values["lat"], values["lon"], values["cog"], values["sog"]
    if not -90 <= lat <= 90:
        return None, f"lat out of range: {lat}"
    if not -180 <= lon <= 180:
        return None, f"lon out of range: {lon}"
    if not 0 <= cog <= 360:
        return None, f"cog out of range: {cog}"
    if sog < 0:
        return None, f"sog out of range: {sog}"
    return (lat, lon, cog, sog), None


def parse_position_payload(data):
    fix, _ = inspect_position_payload(data)
    return fix


def make_mqtt_client(nav, engage_state, route_state, manual_state, pulse_mode_state):
    if mqtt is None:
        raise RuntimeError("paho-mqtt is required for MQTT mode; unset AUTOPILOT_MQTT_* for direct HTTP mode")
    host = os.environ["AUTOPILOT_MQTT_HOST"]
    port = int(os.environ.get("AUTOPILOT_MQTT_PORT", "443"))
    username = os.environ["AUTOPILOT_MQTT_USERNAME"]
    with open(os.environ["AUTOPILOT_MQTT_PASSWORD_FILE"]) as fh:
        password = fh.read().strip()

    client = mqtt.Client(
        client_id=f"gps-only-pilot-{username}",
        transport="websockets",
        protocol=mqtt.MQTTv311,
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    )
    client.tls_set(cert_reqs=ssl.CERT_REQUIRED)  # real Let's Encrypt cert; default CAs suffice
    client.username_pw_set(username, password)
    client.message_callback_add(GPS_TOPIC, nav.on_message)
    client.message_callback_add(ENGAGE_TOPIC, engage_state.on_message)
    client.message_callback_add(ROUTE_TOPIC, route_state.on_message)
    client.message_callback_add(LEG_TOPIC, route_state.on_leg_message)
    client.message_callback_add(MANUAL_TOPIC, manual_state.on_message)
    client.message_callback_add(PULSE_MODE_TOPIC, pulse_mode_state.on_message)

    def on_connect(c, *_a):
        c.subscribe(GPS_TOPIC, qos=1)
        c.subscribe(ENGAGE_TOPIC, qos=1)
        c.subscribe(ROUTE_TOPIC, qos=1)
        c.subscribe(LEG_TOPIC, qos=1)
        c.subscribe(MANUAL_TOPIC, qos=1)
        c.subscribe(PULSE_MODE_TOPIC, qos=1)

    client.on_connect = on_connect
    client.reconnect_delay_set(min_delay=2, max_delay=30)
    client.connect(host, port, keepalive=30)
    return client


def load_route(path):
    data = json.loads(Path(path).read_text())
    if "waypoints" not in data:
        raise ValueError(f"{path} is missing waypoints")
    waypoints = data["waypoints"]
    arrival_radius_m = data.get("arrival_radius_m", 30.0)
    return route_module.Route(waypoints, arrival_radius_m=arrival_radius_m)


def drive_pulse(driver, sign, duration_s):
    end = time.monotonic() + duration_s
    try:
        while time.monotonic() < end:
            driver.command(sign)
            time.sleep(min(DRIVE_REFRESH_S, duration_s))
    finally:
        driver.disengage()


def pulse_travel_s(driver, duration_s):
    """How much rudder travel to credit the budget for a pulse we drove for duration_s.

    Not the same as duration_s: the driver ramps its duty from zero on every pulse, so
    part of every pulse moves nothing. See control.delivered_travel_s.

    Falls back to the wall-clock width for any driver that exposes no ramp config --
    a driver with no ramp really does deliver the full width.
    """
    config = getattr(driver, "config", None)
    slew_per_second = getattr(config, "slew_per_second", None)
    max_duty = getattr(config, "max_duty", None)
    if slew_per_second is None or max_duty is None:
        return duration_s
    return control.delivered_travel_s(duration_s, slew_per_second, max_duty)


def gps_reason_detail(reasons, gps_problem, ever_received):
    """Rewrites the bare "gps fix stale" guardrail reason into one that says which
    of three very different faults produced it, since they need opposite fixes:

    - nothing has ever arrived    -> OpenHelm isn't relaying at all (its fast-GPS
      relay only publishes once this controller reports engaged, so this is the
      normal look of a relay that never started)
    - the latest payload was bad  -> the link is fine, a field is missing
    - neither                     -> a real dropout; fixes stopped coming

    Display-only. steer_enabled's pass/fail decision is untouched.
    """
    if ever_received and gps_problem is None:
        return list(reasons)
    replacement = f"gps unusable: {gps_problem}" if gps_problem else "no gps received from openhelm"
    return [replacement if reason == "gps fix stale" else reason for reason in reasons]


def nav_status_fix(nav, now=None):
    """The live GPS fix as the control loop sees it, for HTTP status consumers.

    OpenHelm polls /autopilot/status to show the boat's fix on the Autopilot tab
    before engaging, and -- when the phone sources GPS from the T-Deck -- to use
    the T-Deck's fix as its own position. Reading the same LiveNav the loop
    steers on keeps the two from disagreeing about where the boat is.

    Prefers the steering-grade fix ( underway, COG present); falls back to the
    last known position so a stationary boat (empty COG at the dock) still
    reports lat/lon. cog is None when the receiver has no course.
    """
    if now is None:
        now = time.monotonic()
    out = {
        "gps_source": nav.active_source,
        "gps_problem": nav.gps_problem,
        "lat": None,
        "lon": None,
        "cog": None,
        "sog": None,
        "fix_age_s": None,
    }
    gps = nav.gps
    if gps is not None and gps.received_at is not None and nav.lat is not None:
        out["lat"] = round(nav.lat, 6)
        out["lon"] = round(nav.lon, 6)
        out["cog"] = gps.cog_deg
        out["sog"] = gps.sog_kn
        out["fix_age_s"] = round(now - gps.received_at, 2)
        return out
    pos = nav.last_position
    if pos is not None:
        lat, lon, cog, sog, received_at = pos
        out["lat"] = round(lat, 6)
        out["lon"] = round(lon, 6)
        out["cog"] = cog
        out["sog"] = sog
        out["fix_age_s"] = None if received_at is None else round(now - received_at, 2)
    return out


def build_status(engaged, reasons, travel_budget, error_deg=None, nav_fix=None, manual_direction=None,
                  pulse_mode=None, learned_duration_ms=None):
    return {
        "engaged": engaged,
        "error_deg": error_deg,
        "travel_budget_position_s": travel_budget.position_s,
        "guardrail_reasons": reasons,
        "leg_index": nav_fix.leg_index if nav_fix is not None else None,
        "distance_to_waypoint_m": nav_fix.distance_to_waypoint_m if nav_fix is not None else None,
        "arrived": nav_fix.arrived if nav_fix is not None else False,
        "manual_direction": manual_direction,
        "pulse_mode": pulse_mode,
        "learned_duration_ms": learned_duration_ms,
    }


def manual_command_sign(direction, direction_map):
    turn_right_sign = control.command_sign_for_turn_right(direction_map)
    return turn_right_sign if direction == "starboard" else -turn_right_sign


def apply_manual_jog(driver, direction, direction_map, travel_budget):
    sign = manual_command_sign(direction, direction_map)
    if travel_budget.at_limit(sign):
        driver.disengage()
        print(f"autopilot: manual {direction} blocked by travel budget")
        return
    duration_s = travel_budget.allowed_pulse_s(sign, MANUAL_PULSE_S)
    if duration_s <= 0:
        driver.disengage()
        return
    drive_pulse(driver, sign, duration_s)
    travel_budget.apply_pulse(sign, pulse_travel_s(driver, duration_s))


def run(nav, driver, route_state, engage_state, manual_state, direction_map, guardrail_config, steering_config,
        travel_budget, publish_status, response_model=None, pulse_mode_state=None, observation_tracker=None,
        learned_calibration_path=None, response_log_path=None, seed_bootstrap=None):
    print("autopilot: waiting for first GPS fix...")
    last_disengaged_log = 0.0
    turn_rate_tracker = control.TurnRateTracker()
    cog_smoother = control.CogSmoother()
    helm_trim = control.HelmTrim()
    # Outer-loop output, held between fixes so the inner loop has something to drive to.
    rudder_target_s = 0.0
    turn_rate_dps = 0.0
    smoothed_cog_deg = None
    last_fix_at_s = None
    recent_observed_dps_history = []
    try:
        while True:
            got_fix = nav.wait_for_new_fix(timeout=CONTROL_TICK_S)
            gps, heartbeat, lat, lon = nav.snapshot()
            engaged = engage_state.engaged
            # Read live every tick, not captured once at startup, so OpenHelm
            # can switch drive modes on the fly without restarting the process.
            pulse_mode = pulse_mode_state.mode if pulse_mode_state is not None else "fixed"
            now = time.monotonic()

            if observation_tracker is not None and gps is not None:
                outcome = observation_tracker.observe_fix(cog_deg=gps.cog_deg, at_s=now)
                if outcome is not None and response_model is None and seed_bootstrap is not None:
                    seed = seed_bootstrap.observe(
                        pulse_ms=outcome.pulse_ms, observed_dps=outcome.observed_dps
                    )
                    if seed is None:
                        print(
                            f"autopilot: seeding response model -- "
                            f"{seed_bootstrap.samples_collected}/{DEFAULT_SEED_SAMPLES} pulses observed"
                        )
                    else:
                        seed_a, seed_b = seed
                        response_model = (
                            learn.load_learned_calibration(learned_calibration_path, seed_a, seed_b)
                            if learned_calibration_path is not None
                            else learn.ResponseModel(a=seed_a, b=seed_b, seed_a=seed_a, seed_b=seed_b)
                        )
                        print(
                            f"autopilot: response model seeded from observed COG "
                            f"(a={seed_a:.6g} deg/s per ms) -- learned pulse mode available"
                        )
                    outcome = None  # consumed by the bootstrap, not a learning step
                if outcome is not None and response_model is not None:
                    weight = learn.outlier_weight(outcome.observed_dps, recent_observed_dps_history)
                    response_model.update(
                        predicted_dps=outcome.predicted_dps,
                        observed_dps=outcome.observed_dps,
                        pulse_ms=outcome.pulse_ms,
                        sog_kn=outcome.sog_kn,
                        learning_rate=RESPONSE_LEARNING_RATE,
                        weight=weight,
                    )
                    recent_observed_dps_history.append(outcome.observed_dps)
                    del recent_observed_dps_history[:-OUTLIER_HISTORY_LEN]
                    if learned_calibration_path is not None:
                        learn.save_learned_calibration(learned_calibration_path, response_model)
                    if response_log_path is not None:
                        learn.log_pulse_outcome(
                            response_log_path,
                            pulse_ms=outcome.pulse_ms,
                            sog_kn=outcome.sog_kn,
                            predicted_dps=outcome.predicted_dps,
                            observed_dps=outcome.observed_dps,
                            drove=pulse_mode,
                        )

            nav_intent = None
            nav_fix = None
            route = route_state.current
            if route is not None and lat is not None and lon is not None:
                fix = route.update(lat, lon)
                nav_fix = fix
                nav_intent = control.NavIntent(
                    bearing_to_waypoint_deg=fix.bearing_to_waypoint_deg,
                    route_bearing_deg=fix.route_bearing_deg,
                    cross_track_error_m=fix.cross_track_error_m,
                    received_at=now,
                )

            manual_direction = manual_state.current(now)
            if manual_direction is not None:
                apply_manual_jog(driver, manual_direction, direction_map, travel_budget)
                publish_status(build_status(engaged, [], travel_budget, nav_fix=nav_fix, manual_direction=manual_direction))
                continue

            enabled, reasons = control.steer_enabled(now, gps, nav_intent, heartbeat, engaged, guardrail_config)
            reasons = gps_reason_detail(reasons, nav.gps_problem, ever_received=heartbeat is not None)

            if nav_fix is not None and nav_fix.arrived:
                driver.disengage()
                publish_status(build_status(engaged, reasons, travel_budget, nav_fix=nav_fix))
                print("autopilot: arrived at final waypoint -- disengaged")
                continue

            if not enabled:
                driver.disengage()
                last_fix_at_s = None
                if not engaged:
                    # Drop learned trim only when the *operator* disengages -- it belongs
                    # to the conditions we were steering in. Keyed on `engaged`, not
                    # `enabled`: on the 2026-08-04 trial resetting on any guardrail trip
                    # meant a momentary GPS dropout threw away a minute of accumulated
                    # trim, and she fell 48deg off course in the next 75 seconds.
                    helm_trim.reset()
                    # Disengaged means the operator has the helm, and they centre it.
                    # Without this the dead-reckoned position carries whatever offset it
                    # had into the next engage -- on the 2026-08-04 trial it re-engaged
                    # believing 30+ seconds of helm were already wound on. Deliberately
                    # keyed on `engaged` rather than `enabled`: a momentary GPS dropout
                    # must not zero the estimate while the helm is genuinely over.
                    travel_budget.position_s = 0.0
                publish_status(build_status(engaged, reasons, travel_budget, nav_fix=nav_fix))
                if not got_fix and now - last_disengaged_log >= 3.0:
                    print(f"autopilot: disengaged -- {', '.join(reasons)}")
                    last_disengaged_log = now
                continue

            # Smooth COG once per fix and steer on that, not on the raw value: the raw
            # 1 Hz course wobbles a couple of degrees on noise alone, and unfiltered it
            # jittered both the heading error and the damping term.
            if got_fix:
                smoothed_cog_deg = cog_smoother.update(gps.cog_deg)
                turn_rate_dps = turn_rate_tracker.update(smoothed_cog_deg, now)

            error_deg = control.steering_error(
                gps, nav_intent, steering_config, cog_deg=smoothed_cog_deg
            )

            # OUTER loop, once per GPS fix: decide where the helm should be. Only new
            # position data can change that, so gating it here is what stopped the
            # original bang-bang -- ten pulses fired off one stale error reading.
            if got_fix:
                max_rudder_s = control.max_rudder_s_for(
                    travel_budget.hard_over_seconds, steering_config
                )
                # Auto-trim: carry the steady pull of sail, leeway or current as standing
                # helm, so the error can reach zero instead of settling wherever the
                # proportional term happens to balance the force. Held off while the helm
                # is saturated, or it winds up through a turn and slams over on exit.
                dt_s = 0.0 if last_fix_at_s is None else now - last_fix_at_s
                last_fix_at_s = now
                trim_s = helm_trim.update(
                    error_deg,
                    dt_s,
                    max_trim_s=steering_config.max_trim_fraction * max_rudder_s,
                    saturated=abs(rudder_target_s) >= max_rudder_s - 1e-6,
                )
                rudder_target_s = control.target_rudder_s(
                    error_deg,
                    turn_rate_dps,
                    max_rudder_s,
                    direction_map,
                    steering_config,
                    trim_s=trim_s,
                )

            # INNER loop, every tick: actually get the helm there. Moving the rudder needs
            # no new GPS, and gating it to 1 Hz capped the slew at max_pulse_ms per second
            # -- a few seconds of demanded rudder took half a minute to wind on, by which
            # point the boat had swung 60deg past and the demand had reversed. That phase
            # lag was the second trial's oscillation.
            fixed_decision = control.rudder_move(
                rudder_target_s, travel_budget.position_s, steering_config
            )

            learned_duration_ms = None
            learned_predicted_dps = None
            if response_model is not None:
                desired_turn_rate_dps = error_deg / max(CONTROL_TICK_S, 1e-6)
                learned_predicted_dps = response_model.predict_turn_rate_dps(pulse_ms=100.0, sog_kn=gps.sog_kn)
                learned_duration_ms = min(
                    response_model.predict_pulse_ms(
                        desired_turn_rate_dps=abs(desired_turn_rate_dps), sog_kn=gps.sog_kn
                    ),
                    MAX_LEARNED_PULSE_MS,
                )

            if pulse_mode == "learned" and response_model is not None:
                if learned_duration_ms is None or learned_duration_ms <= 0:
                    decision = None
                else:
                    turn_right_sign = control.command_sign_for_turn_right(direction_map)
                    sign = turn_right_sign if error_deg > 0 else -turn_right_sign
                    decision = (sign, learned_duration_ms)
            else:
                decision = fixed_decision

            if decision is None:
                publish_status(build_status(
                    engaged, reasons, travel_budget, error_deg=error_deg, nav_fix=nav_fix,
                    pulse_mode=pulse_mode, learned_duration_ms=learned_duration_ms,
                ))
                continue

            sign, duration_ms = decision
            if travel_budget.at_limit(sign):
                print(f"autopilot: travel budget at limit for sign {sign:+d} -- skipping pulse")
                publish_status(build_status(
                    engaged, reasons, travel_budget, error_deg=error_deg, nav_fix=nav_fix,
                    pulse_mode=pulse_mode, learned_duration_ms=learned_duration_ms,
                ))
                continue

            requested_duration_s = duration_ms / 1000.0
            duration_s = travel_budget.allowed_pulse_s(sign, requested_duration_s)
            if duration_s <= 0:
                print(f"autopilot: travel budget at limit for sign {sign:+d} -- skipping pulse")
                publish_status(build_status(
                    engaged, reasons, travel_budget, error_deg=error_deg, nav_fix=nav_fix,
                    pulse_mode=pulse_mode, learned_duration_ms=learned_duration_ms,
                ))
                continue
            drive_pulse(driver, sign, duration_s)
            travel_budget.apply_pulse(sign, pulse_travel_s(driver, duration_s))
            actual_duration_ms = round(duration_s * 1000)

            # Also runs while unseeded: these are exactly the pulses the bootstrap
            # needs to watch.
            if observation_tracker is not None:
                observation_tracker.pulse_fired(
                    pulse_ms=actual_duration_ms,
                    sog_kn=gps.sog_kn,
                    cog_deg=gps.cog_deg,
                    fired_at_s=now,
                    predicted_dps=learned_predicted_dps,
                )

            publish_status(build_status(
                engaged, reasons, travel_budget, error_deg=error_deg, nav_fix=nav_fix,
                pulse_mode=pulse_mode, learned_duration_ms=learned_duration_ms,
            ))
            # Full breakdown: error alone hid whether the *target* was sane. brg is the
            # leg's bearing, tgt that bearing after the XTE nudge, and error is tgt-cog.
            print(
                f"autopilot: error={error_deg:+.1f}deg turn={turn_rate_dps:+.1f}dps "
                f"cog={gps.cog_deg:.0f} brg={nav_intent.route_bearing_deg:.0f} "
                f"btw={nav_intent.bearing_to_waypoint_deg:.0f} "
                f"tgt={control.target_track(nav_intent, steering_config):.0f} "
                f"xte={nav_intent.cross_track_error_m:+.0f}m "
                f"pulse={sign:+d}x{actual_duration_ms}ms "
                f"budget={travel_budget.position_s:+.2f}s trim={helm_trim.trim_s:+.2f} "
                f"mode={pulse_mode}"
            )
    finally:
        driver.disengage()


def dispatch_publish(nav, engage_state, route_state, manual_state, pulse_mode_state, topic, payload, retain=False):
    msg = SimpleNamespace(topic=topic, payload=json.dumps(payload).encode("utf-8"), retain=retain)
    if topic == GPS_TOPIC:
        nav.on_message(None, None, msg)
    elif topic == ENGAGE_TOPIC:
        engage_state.on_message(None, None, msg)
    elif topic == ROUTE_TOPIC:
        if not route_state.on_message(None, None, msg):
            raise ValueError("route rejected")
    elif topic == LEG_TOPIC:
        if not route_state.on_leg_message(None, None, msg):
            raise ValueError("leg rejected")
    elif topic == MANUAL_TOPIC:
        manual_state.on_message(None, None, msg)
    elif topic == PULSE_MODE_TOPIC:
        pulse_mode_state.on_message(None, None, msg)
    else:
        raise ValueError(f"unknown autopilot topic: {topic}")


def make_direct_handler(nav, engage_state, route_state, manual_state, pulse_mode_state, status_state, on_publish=None):
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, code, payload):
            data = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("access-control-allow-origin", "*")
            self.send_header("access-control-allow-methods", "GET,POST,OPTIONS")
            self.send_header("access-control-allow-headers", "content-type")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_OPTIONS(self):
            self._send_json(200, {"ok": True})

        def do_GET(self):
            if self.path == "/autopilot/status":
                # Merge the live fix in at serve time rather than threading it
                # through every build_status call site: the wire schema stays
                # stable, and the fix is always as fresh as the request.
                snap = dict(status_state.snapshot())
                snap.update(nav_status_fix(nav))
                self._send_json(200, snap)
                return
            self._send_json(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/autopilot/publish":
                self._send_json(404, {"error": "not found"})
                return
            try:
                size = int(self.headers.get("content-length", "0"))
                data = json.loads(self.rfile.read(size).decode("utf-8"))
                dispatch_publish(
                    nav,
                    engage_state,
                    route_state,
                    manual_state,
                    pulse_mode_state,
                    data["topic"],
                    data.get("payload"),
                    bool(data.get("retain", False)),
                )
                if on_publish is not None:
                    on_publish(data["topic"], data.get("payload"))
            except Exception as e:
                self._send_json(400, {"error": str(e)})
                return
            self._send_json(200, {"ok": True})

        def log_message(self, _fmt, *_args):
            return

    return Handler


class DirectControlServer:
    def __init__(self, port, nav, engage_state, route_state, manual_state, pulse_mode_state, status_state,
                 host="0.0.0.0", on_publish=None):
        handler = make_direct_handler(nav, engage_state, route_state, manual_state, pulse_mode_state, status_state, on_publish)
        self._server = ThreadingHTTPServer((host, port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def start(self):
        self._thread.start()
        print(f"autopilot: direct HTTP test server listening on port {self._server.server_port}")

    def stop(self):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)


class NmeaUdpListener:
    """Receives the T-Deck's broadcast NMEA and feeds it to LiveNav.

    The T-Deck sends the receiver's own sentences unmodified, so this consumes
    standard NMEA-0183 -- any other GPS that broadcasts to the same port works
    without changes here.

    Binds to all interfaces rather than the broadcast address itself: the socket
    has to accept datagrams addressed to the subnet broadcast, and on some stacks
    binding the broadcast address directly will not receive them.
    """

    def __init__(self, nav, port=DEFAULT_NMEA_UDP_PORT, source=SOURCE_TDECK):
        self.nav = nav
        self.port = port
        self.source = source
        self._sock = None
        self._thread = None
        self._stop = threading.Event()
        # Diagnostics: a feed that arrives but never parses looks identical to no
        # feed at all from the control loop's side.
        self.datagrams = 0
        self.fixes = 0
        self.rejected = 0

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self.port))
        # Without a timeout the thread would block in recvfrom forever and stop()
        # could never join it.
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f"autopilot: listening for NMEA on udp/{self.port}")

    def _run(self):
        while not self._stop.is_set():
            try:
                payload, _addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self.handle_datagram(payload)

    def handle_datagram(self, payload):
        """Parses one datagram. Separated from the socket so tests can drive it."""
        self.datagrams += 1
        for line in nmea.split_datagram(payload):
            fix = nmea.parse_rmc(line)
            if fix is None:
                continue
            if fix.cog_deg is None:
                # Stationary: the receiver has a position but no course. Keep the
                # position for status consumers (a phone sourcing GPS from the
                # T-Deck still wants to know where it is at the dock), but report
                # the fix as steering-unusable rather than inventing a heading --
                # the guardrails already refuse to steer without COG.
                self.nav.record_position(fix.lat, fix.lon, None, fix.sog_kn)
                self.nav.apply_fix(None, "cog unset (stationary)", self.source)
                self.rejected += 1
                continue
            if self.nav.apply_fix((fix.lat, fix.lon, fix.cog_deg, fix.sog_kn), None, self.source):
                self.fixes += 1

    def stop(self):
        self._stop.set()
        if self._sock is not None:
            self._sock.close()
        if self._thread is not None:
            self._thread.join(timeout=2)


def nmea_udp_port_from_env():
    """Returns the port to listen on, or None to leave the UDP path off."""
    raw = os.environ.get("AUTOPILOT_NMEA_UDP_PORT")
    if raw is None:
        return DEFAULT_NMEA_UDP_PORT
    if raw.strip() == "":
        return None  # explicitly disabled
    port = int(raw)
    if not 1 <= port <= 65535:
        raise ValueError(f"AUTOPILOT_NMEA_UDP_PORT out of range: {port}")
    return port


# Configure the broadcast destination for the intended local network.
DEFAULT_STATUS_UDP_PORT = 10111
DEFAULT_STATUS_UDP_BROADCAST = "255.255.255.255"


class StatusUdpBroadcaster:
    """Pushes autopilot status to the boat LAN as a UDP broadcast, so OpenHelm subscribes
    instead of polling /autopilot/status. Same model as the T-Deck's NMEA broadcast: the Pi is
    the source, every device on the boat AP listens.

    Best-effort by design -- a dropped status packet just means a slightly stale panel until the
    next (the control loop publishes on every change, ~3 Hz when steering). A send failure must
    never stall the control loop. Disabled entirely when AUTOPILOT_STATUS_UDP_PORT is empty.
    """

    def __init__(self, broadcast_addr, port):
        self.broadcast_addr = broadcast_addr
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sent = 0
        self.failed = 0

    def send(self, status):
        payload = json.dumps(status).encode("utf-8")
        try:
            self._sock.sendto(payload, (self.broadcast_addr, self.port))
            self.sent += 1
        except OSError:
            self.failed += 1

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


def status_udp_from_env():
    """Returns (broadcast_addr, port) for the status broadcast, or None to disable it.

    On by default (the boat AP broadcast) so OpenHelm gets status by subscription; set
    AUTOPILOT_STATUS_UDP_PORT="" to force the old HTTP-poll fallback.
    """
    port_raw = os.environ.get("AUTOPILOT_STATUS_UDP_PORT")
    if port_raw is not None and port_raw.strip() == "":
        return None  # explicitly disabled
    port = DEFAULT_STATUS_UDP_PORT if port_raw is None else int(port_raw)
    if not 1 <= port <= 65535:
        raise ValueError(f"AUTOPILOT_STATUS_UDP_PORT out of range: {port}")
    addr = os.environ.get("AUTOPILOT_STATUS_UDP_BROADCAST", DEFAULT_STATUS_UDP_BROADCAST)
    return (addr, port)


def maybe_load_initial_route():
    path = os.environ.get("AUTOPILOT_ROUTE_FILE")
    if not path:
        return None
    if not Path(path).exists():
        print(f"autopilot: bootstrap route {path} not found -- waiting for OpenHelm route push")
        return None
    return load_route(path)


def mqtt_env_configured():
    return all(
        os.environ.get(k)
        for k in ("AUTOPILOT_MQTT_HOST", "AUTOPILOT_MQTT_USERNAME", "AUTOPILOT_MQTT_PASSWORD_FILE")
    )


def unavailable_status(engage_state, route_state, manual_direction=None):
    reasons = [NO_CALIBRATION_REASON]
    if not engage_state.engaged:
        reasons.append("not engaged")
    if route_state.current is None:
        reasons.append("route data stale")
    return {
        "engaged": engage_state.engaged,
        "error_deg": None,
        "travel_budget_position_s": None,
        "guardrail_reasons": reasons,
        "leg_index": None,
        "distance_to_waypoint_m": None,
        "arrived": False,
        "manual_direction": manual_direction,
    }


def manual_direction_from_payload(payload):
    if not isinstance(payload, dict):
        return None
    direction = payload.get("direction")
    if direction not in ("port", "starboard", None):
        raise ValueError(f"invalid manual direction: {direction!r}")
    return direction


def run_direct_without_calibration(direct, status_state, engage_state, route_state, manual_proxy):
    print("autopilot: calibration missing -- direct manual proxy only; auto steering disabled")
    status_state.set(unavailable_status(engage_state, route_state))
    direct.start()
    try:
        while True:
            time.sleep(3600)
    finally:
        try:
            manual_proxy.apply(None)
        except Exception as e:
            print(f"autopilot: manual proxy stop failed -- {e}")
        direct.stop()


def main():
    calibration_path = os.environ.get("AUTOPILOT_CALIBRATION_FILE", "calibration.json")
    calibration_error = None
    try:
        direction_map, hard_over_seconds = control.load_calibration(calibration_path)
    except ValueError as e:
        calibration_error = e
        direction_map, hard_over_seconds = None, None

    route_state = RouteState(maybe_load_initial_route())
    engage_state = EngageState()
    manual_state = ManualState()
    pulse_mode_state = PulseModeState(initial_mode=pulse_mode_from_env())
    status_state = StatusState()
    nav = LiveNav()
    client = make_mqtt_client(nav, engage_state, route_state, manual_state, pulse_mode_state) if mqtt_env_configured() else None
    direct = None
    direct_port = os.environ.get("AUTOPILOT_DIRECT_HTTP_PORT")
    manual_proxy = None
    on_direct_publish = None
    if calibration_error is not None:
        proxy_url = os.environ.get("AUTOPILOT_MANUAL_PROXY_URL")
        if not direct_port or not proxy_url:
            raise calibration_error
        manual_proxy = MotorWebManualProxy(
            proxy_url,
            duty=float(os.environ.get("AUTOPILOT_MANUAL_PROXY_DUTY", "90")),
            starboard_sign=int(os.environ.get("AUTOPILOT_MANUAL_PROXY_STARBOARD_SIGN", "1")),
        )

        def on_direct_publish(topic, payload):
            manual_direction = None
            if topic == MANUAL_TOPIC:
                manual_direction = manual_direction_from_payload(payload)
                manual_proxy.apply(manual_direction)
            status_state.set(unavailable_status(engage_state, route_state, manual_direction))

    if direct_port:
        direct = DirectControlServer(
            int(direct_port),
            nav,
            engage_state,
            route_state,
            manual_state,
            pulse_mode_state,
            status_state,
            on_publish=on_direct_publish,
        )
    if client is None and direct is None:
        raise ValueError("configure MQTT credentials or AUTOPILOT_DIRECT_HTTP_PORT")

    # The T-Deck's NMEA feed is a GPS source only -- it carries no engage/route
    # commands, so it never replaces the need for MQTT or the direct server.
    nmea_port = nmea_udp_port_from_env()
    nmea_listener = NmeaUdpListener(nav, nmea_port) if nmea_port else None
    # Status broadcast: OpenHelm subscribes to this instead of polling /autopilot/status.
    # Same boat-LAN-broadcast model as the T-Deck's NMEA. None falls back to the HTTP poll.
    status_udp = status_udp_from_env()
    status_broadcaster = StatusUdpBroadcaster(*status_udp) if status_udp else None

    driver = None
    loop_started = False
    try:
        if calibration_error is not None:
            if client is not None:
                raise calibration_error
            run_direct_without_calibration(direct, status_state, engage_state, route_state, manual_proxy)
            return 0
        if direct is not None:
            direct.start()
        if nmea_listener is not None:
            nmea_listener.start()
        if client is not None:
            client.loop_start()
            loop_started = True
        driver = BTS7960ServoDriver(BTS7960Config())
        travel_budget = control.TravelBudget(hard_over_seconds=hard_over_seconds)

        learned_calibration_path = os.environ.get("AUTOPILOT_LEARNED_CALIBRATION_FILE", "learned_calibration.json")
        response_log_path = os.environ.get("AUTOPILOT_RESPONSE_LOG_FILE", "response_log.jsonl")
        response_model = None
        seed_bootstrap = None
        observation_tracker = learn.PulseObservationTracker(fixes_to_observe=DEFAULT_OBSERVE_FIXES)
        try:
            raw_calibration = json.loads(Path(calibration_path).read_text())
            seed_a, seed_b = learn.seed_response_model(raw_calibration)
            response_model = learn.load_learned_calibration(learned_calibration_path, seed_a, seed_b)
        except ValueError as e:
            # No rudder-angle table -- seed from the boat's own COG response instead.
            # Fixed pulses drive until then, so this is a delay, not a failure.
            seed_bootstrap = learn.ResponseSeedBootstrap(samples_needed=DEFAULT_SEED_SAMPLES)
            print(
                f"autopilot: no bench calibration ({e}) -- seeding the response model "
                f"from observed COG over the next {DEFAULT_SEED_SAMPLES} pulses; "
                "fixed pulses until then"
            )

        def publish_status(status):
            # Merge the live fix in here (the single funnel every publish goes through) so the
            # UDP broadcast + MQTT carry it too -- not just the HTTP do_GET, which merges at serve
            # time. Without this a subscriber on the boat AP saw status with lat/lon null ("no
            # fix") even though the Pi has the fix.
            full = dict(status)
            full.update(nav_status_fix(nav))
            status_state.set(full)
            if status_broadcaster is not None:
                status_broadcaster.send(full)
            if client is not None:
                client.publish(STATUS_TOPIC, json.dumps(full), qos=1)

        run(
            nav,
            driver,
            route_state,
            engage_state,
            manual_state,
            direction_map,
            control.GuardrailConfig(),
            control.SteeringConfig(),
            travel_budget,
            publish_status,
            response_model=response_model,
            pulse_mode_state=pulse_mode_state,
            observation_tracker=observation_tracker,
            learned_calibration_path=learned_calibration_path,
            response_log_path=response_log_path,
            seed_bootstrap=seed_bootstrap,
        )
    except KeyboardInterrupt:
        print("\nautopilot: interrupted -- stopping.")
    finally:
        if driver is not None:
            driver.close()
        if loop_started:
            client.loop_stop()
        if client is not None:
            client.disconnect()
        if direct is not None:
            direct.stop()
        if nmea_listener is not None:
            nmea_listener.stop()
        if status_broadcaster is not None:
            status_broadcaster.close()


if __name__ == "__main__":
    sys.exit(main())
