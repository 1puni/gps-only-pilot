import json
import socket
import time
from types import SimpleNamespace

from urllib import error, request

import pytest

import autopilot
import learn
from autopilot import EngageState, LiveNav, ManualState, RouteState, StatusState
from control import GuardrailConfig, GpsFix, Heartbeat, SteeringConfig, TravelBudget
from route import Route


def make_msg(topic, payload_dict, retain=False):
    return SimpleNamespace(topic=topic, payload=json.dumps(payload_dict).encode("utf-8"), retain=retain)


GPS_TOPIC = "openhelm/autopilot/vessel/gps"


def test_ignores_retained_seed_messages():
    nav = LiveNav()
    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187, "ts": 1}, retain=True),
    )
    assert nav.gps is None
    assert nav.heartbeat is None
    assert nav.lat is None


def test_valid_fix_updates_gps_and_lat_lon():
    nav = LiveNav()
    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187, "ts": 1}),
    )
    assert nav.gps.cog_deg == 187
    assert nav.gps.sog_kn == 4.2
    assert nav.lat == 57.7
    assert nav.lon == 11.9


def test_null_cog_leaves_gps_unusable_but_still_a_heartbeat():
    nav = LiveNav()
    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": None, "cog": None, "ts": 1}),
    )
    assert nav.gps is None
    assert nav.heartbeat is not None


def test_invalid_fix_clears_previous_position_but_keeps_heartbeat():
    nav = LiveNav()
    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187, "ts": 1}),
    )
    assert nav.gps is not None

    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 91.0, "lon": 11.9, "sog": 4.2, "cog": 187, "ts": 2}),
    )

    assert nav.gps is None
    assert nav.lat is None
    assert nav.lon is None
    assert nav.heartbeat is not None


def test_inspect_position_payload_names_the_unusable_field():
    assert autopilot.inspect_position_payload(
        {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187}
    ) == ((57.7, 11.9, 187.0, 4.2), None)

    _, problem = autopilot.inspect_position_payload({"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": None})
    assert problem == "cog is None"
    _, problem = autopilot.inspect_position_payload({"lat": 57.7, "lon": 11.9, "sog": None, "cog": 187})
    assert problem == "sog is None"
    _, problem = autopilot.inspect_position_payload({"lat": 91.0, "lon": 11.9, "sog": 4.2, "cog": 187})
    assert problem == "lat out of range: 91.0"


def test_unusable_fix_logs_the_offending_field(capsys):
    nav = LiveNav()
    nav.on_message(None, None, make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": None}))
    assert "cog is None" in capsys.readouterr().out


def test_repeated_unusable_fixes_are_rate_limited(capsys):
    # 1 Hz of null-COG fixes must not fill the runit log with one line per second.
    nav = LiveNav()
    msg = make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": None})
    for _ in range(10):
        nav.on_message(None, None, msg)
    assert capsys.readouterr().out.count("cog is None") == 1


def test_gps_reason_detail_distinguishes_never_received_from_rejected():
    # "gps fix stale" alone can't tell the helm whether OpenHelm is silent or
    # whether its fixes are arriving and being thrown away -- very different faults.
    assert autopilot.gps_reason_detail(
        ["gps fix stale"], gps_problem=None, ever_received=False
    ) == ["no gps received from openhelm"]

    assert autopilot.gps_reason_detail(
        ["gps fix stale"], gps_problem="cog is None", ever_received=True
    ) == ["gps unusable: cog is None"]

    assert autopilot.gps_reason_detail(
        ["gps fix stale"], gps_problem=None, ever_received=True
    ) == ["gps fix stale"]


def test_gps_reason_detail_leaves_other_reasons_alone():
    assert autopilot.gps_reason_detail(
        ["not engaged", "gps fix stale", "route data stale"],
        gps_problem="sog is None", ever_received=True,
    ) == ["not engaged", "gps unusable: sog is None", "route data stale"]

    assert autopilot.gps_reason_detail(
        ["route data stale"], gps_problem=None, ever_received=False
    ) == ["route data stale"]


def test_live_nav_tracks_current_gps_problem():
    nav = LiveNav()
    assert nav.gps_problem is None

    nav.on_message(None, None, make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": None}))
    assert nav.gps_problem == "cog is None"

    nav.on_message(None, None, make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187}))
    assert nav.gps_problem is None


def test_wait_for_new_fix_signals_after_on_message():
    nav = LiveNav()
    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187, "ts": 1}),
    )
    assert nav.wait_for_new_fix(timeout=0) is True
    # Event is cleared after being consumed once.
    assert nav.wait_for_new_fix(timeout=0) is False


def test_engage_state_defaults_to_disengaged():
    state = EngageState()
    assert state.engaged is False


def test_engage_state_latches_true_on_engaged_message():
    state = EngageState()
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/engage", {"engaged": True}))
    assert state.engaged is True


def test_engage_state_latches_false_on_disengaged_message():
    state = EngageState()
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/engage", {"engaged": True}))
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/engage", {"engaged": False}))
    assert state.engaged is False


def test_engage_state_ignores_malformed_payload():
    state = EngageState()
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/engage", {"engaged": "yes"}))
    assert state.engaged is False


def test_pulse_mode_state_defaults_to_fixed():
    state = autopilot.PulseModeState()
    assert state.mode == "fixed"


def test_pulse_mode_state_can_seed_a_different_initial_mode():
    state = autopilot.PulseModeState(initial_mode="learned")
    assert state.mode == "learned"


def test_pulse_mode_state_latches_on_valid_message():
    state = autopilot.PulseModeState()
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/pulse_mode", {"mode": "learned"}))
    assert state.mode == "learned"
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/pulse_mode", {"mode": "fixed"}))
    assert state.mode == "fixed"


def test_pulse_mode_state_ignores_malformed_payload():
    state = autopilot.PulseModeState()
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/pulse_mode", {"mode": "sideways"}))
    assert state.mode == "fixed"
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/pulse_mode", {}))
    assert state.mode == "fixed"


def test_route_state_starts_with_initial_route():
    initial = Route([(0, 0), (0, 1)])
    state = RouteState(initial)
    assert state.current is initial


def test_route_state_replaces_route_on_valid_push():
    state = RouteState(Route([(0, 0), (0, 1)]))
    accepted = state.on_message(
        None, None,
        make_msg(
            "openhelm/autopilot/vessel/route",
            {"waypoints": [[1, 1], [2, 2], [3, 3]], "arrival_radius_m": 50},
        ),
    )
    assert state.current.waypoints == [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
    assert state.current.arrival_radius_m == 50
    assert state.current.leg_index == 0
    assert accepted is True


def test_route_state_keeps_previous_route_on_invalid_push():
    initial = Route([(0, 0), (0, 1)])
    state = RouteState(initial)
    accepted = state.on_message(
        None, None,
        make_msg("openhelm/autopilot/vessel/route", {"waypoints": [[0, 0]]}),  # only 1 waypoint
    )
    assert state.current is initial
    assert accepted is False


def test_route_state_ignores_message_with_no_waypoints_key():
    initial = Route([(0, 0), (0, 1)])
    state = RouteState(initial)
    accepted = state.on_message(None, None, make_msg("openhelm/autopilot/vessel/route", {"arrival_radius_m": 50}))
    assert state.current is initial
    assert accepted is False


def test_route_state_defaults_to_leg_zero_when_openhelm_sends_none():
    # An OpenHelm build that doesn't send leg_index must keep working unchanged.
    state = RouteState()
    state.on_message(
        None, None,
        make_msg("openhelm/autopilot/vessel/route", {"waypoints": [[0, 0], [0, 1], [1, 1]]}),
    )
    assert state.current.leg_index == 0


def test_route_state_honours_pushed_leg_index():
    state = RouteState()
    accepted = state.on_message(
        None, None,
        make_msg(
            "openhelm/autopilot/vessel/route",
            {"waypoints": [[0, 0], [0, 1], [1, 1]], "leg_index": 1},
        ),
    )
    assert accepted is True
    assert state.current.leg_index == 1


def test_route_state_rejects_push_with_out_of_range_leg_index():
    initial = Route([(0, 0), (0, 1)])
    state = RouteState(initial)
    accepted = state.on_message(
        None, None,
        make_msg(
            "openhelm/autopilot/vessel/route",
            {"waypoints": [[0, 0], [0, 1], [1, 1]], "leg_index": 9},
        ),
    )
    assert accepted is False
    assert state.current is initial  # previous route kept


def test_leg_message_sets_the_active_leg():
    state = RouteState(Route([(0, 0), (0, 1), (1, 1)]))
    accepted = state.on_leg_message(
        None, None, make_msg("openhelm/autopilot/vessel/leg", {"leg_index": 1})
    )
    assert accepted is True
    assert state.current.leg_index == 1


def test_leg_message_may_move_backward():
    # "Set as next mark" is the operator's authority -- it overrides our own advance.
    state = RouteState(Route([(0, 0), (0, 1), (1, 1)], leg_index=1))
    accepted = state.on_leg_message(
        None, None, make_msg("openhelm/autopilot/vessel/leg", {"leg_index": 0})
    )
    assert accepted is True
    assert state.current.leg_index == 0


def test_leg_message_rejected_when_no_route_loaded():
    state = RouteState()
    accepted = state.on_leg_message(
        None, None, make_msg("openhelm/autopilot/vessel/leg", {"leg_index": 0})
    )
    assert accepted is False


def test_leg_message_rejected_when_out_of_range_and_keeps_current_leg():
    state = RouteState(Route([(0, 0), (0, 1), (1, 1)], leg_index=1))
    accepted = state.on_leg_message(
        None, None, make_msg("openhelm/autopilot/vessel/leg", {"leg_index": 7})
    )
    assert accepted is False
    assert state.current.leg_index == 1


def test_leg_message_rejected_when_index_missing():
    state = RouteState(Route([(0, 0), (0, 1)]))
    accepted = state.on_leg_message(
        None, None, make_msg("openhelm/autopilot/vessel/leg", {})
    )
    assert accepted is False


def test_manual_state_latches_and_expires(monkeypatch):
    times = iter([100.0, 100.5, 101.0])
    monkeypatch.setattr(autopilot.time, "monotonic", lambda: next(times))
    state = ManualState(max_age_s=0.7)
    state.on_message(None, None, make_msg("openhelm/autopilot/vessel/manual", {"direction": "port"}))
    assert state.current(100.5) == "port"
    assert state.current(101.0) is None


def test_dispatch_publish_routes_direct_http_payloads():
    nav = LiveNav()
    engage = EngageState()
    route_state = RouteState()
    manual = ManualState()
    pulse_mode_state = autopilot.PulseModeState()

    autopilot.dispatch_publish(nav, engage, route_state, manual, pulse_mode_state, autopilot.ENGAGE_TOPIC, {"engaged": True})
    autopilot.dispatch_publish(nav, engage, route_state, manual, pulse_mode_state, autopilot.ROUTE_TOPIC, {"waypoints": [[0, 0], [0, 1]]})
    autopilot.dispatch_publish(nav, engage, route_state, manual, pulse_mode_state, autopilot.MANUAL_TOPIC, {"direction": "starboard"})
    autopilot.dispatch_publish(nav, engage, route_state, manual, pulse_mode_state, autopilot.GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "cog": 1, "sog": 3})
    autopilot.dispatch_publish(nav, engage, route_state, manual, pulse_mode_state, autopilot.PULSE_MODE_TOPIC, {"mode": "learned"})

    assert engage.engaged is True
    assert route_state.current.waypoints == [(0.0, 0.0), (0.0, 1.0)]
    assert manual.current(time.monotonic()) == "starboard"
    assert nav.gps.sog_kn == 3
    assert pulse_mode_state.mode == "learned"


def test_dispatch_publish_applies_leg_topic():
    route_state = RouteState(Route([(0, 0), (0, 1), (1, 1)]))
    autopilot.dispatch_publish(
        LiveNav(), EngageState(), route_state, ManualState(), autopilot.PulseModeState(),
        autopilot.LEG_TOPIC, {"leg_index": 1},
    )
    assert route_state.current.leg_index == 1


def test_dispatch_publish_raises_on_rejected_leg():
    # A 400 back to the phone, rather than silently steering at the wrong mark.
    route_state = RouteState(Route([(0, 0), (0, 1)]))
    with pytest.raises(ValueError):
        autopilot.dispatch_publish(
            LiveNav(), EngageState(), route_state, ManualState(), autopilot.PulseModeState(),
            autopilot.LEG_TOPIC, {"leg_index": 4},
        )


def test_motor_web_manual_proxy_maps_manual_directions(monkeypatch):
    calls = []

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return b'{"ok": true}'

    def fake_urlopen(req, timeout):
        calls.append((req.full_url, json.loads(req.data.decode("utf-8")), timeout))
        return FakeResponse()

    monkeypatch.setattr(autopilot.request, "urlopen", fake_urlopen)
    proxy = autopilot.MotorWebManualProxy("http://127.0.0.1:8080/", duty=42, starboard_sign=1)

    proxy.apply("starboard")
    proxy.apply("port")
    proxy.apply(None)

    assert calls == [
        ("http://127.0.0.1:8080/api/drive", {"direction": 1, "duty": 42.0}, 0.75),
        ("http://127.0.0.1:8080/api/drive", {"direction": -1, "duty": 42.0}, 0.75),
        ("http://127.0.0.1:8080/api/stop", {}, 0.75),
    ]


def test_load_route_rejects_missing_waypoints(tmp_path):
    path = tmp_path / "route.json"
    path.write_text('{"arrival_radius_m": 30}')
    with pytest.raises(ValueError):
        autopilot.load_route(path)


class OneShotNav:
    def __init__(self):
        self.calls = 0
        now = time.monotonic()
        self.gps = GpsFix(cog_deg=0, sog_kn=4, received_at=now)
        self.heartbeat = Heartbeat(received_at=now)
        self.gps_problem = None

    def wait_for_new_fix(self, timeout):
        self.calls += 1
        if self.calls > 1:
            raise KeyboardInterrupt
        return True

    def snapshot(self):
        return self.gps, self.heartbeat, 0.0, 0.5


class FakeRoute:
    def update(self, lat, lon):
        return SimpleNamespace(
            arrived=False,
            route_bearing_deg=30,
            bearing_to_waypoint_deg=30,
            cross_track_error_m=0,
            leg_index=0,
            distance_to_waypoint_m=1234.0,
        )


class FakeDriver:
    def __init__(self):
        self.disengaged = 0

    def disengage(self):
        self.disengaged += 1


def test_run_unwinds_a_rudder_wound_out_past_its_envelope(monkeypatch):
    """A rudder already near the stops must be brought back, not pushed further.

    The old increment law would add another pulse toward the stop whatever the rudder
    was already doing -- that wound the helm past half lock on the 2026-08-04 trial and
    turned the boat a full 360, because nothing could unwind it. The position law caps
    usable helm at max_rudder_fraction of hard-over and drives *back* toward it.
    """
    pulses = []

    def fake_drive_pulse(_driver, sign, duration_s):
        pulses.append((sign, duration_s))

    monkeypatch.setattr(autopilot, "drive_pulse", fake_drive_pulse)

    # 4.85s of helm on a 5s hard-over, with a 25% envelope configured (1.25s) -- far
    # outside it. Set explicitly so this test survives retuning of the default.
    unwind_config = SteeringConfig(max_rudder_fraction=0.25)
    travel_budget = TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 5.0}, position_s=4.85)
    statuses = []
    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            OneShotNav(),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            unwind_config,
            travel_budget,
            statuses.append,
        )

    # Error is +30deg (wants more starboard), but the helm is already wound out well
    # past what it may use -- so it comes back regardless of the error.
    assert len(pulses) == 1
    sign, duration_s = pulses[0]
    assert sign == -1, "must unwind toward centre, not add more helm toward the stop"
    assert travel_budget.position_s < 4.85
    assert statuses[-1]["engaged"] is True
    assert statuses[-1]["distance_to_waypoint_m"] == 1234.0


def test_run_does_not_steer_when_not_engaged(monkeypatch):
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda *a: pulses.append(a))

    statuses = []
    driver = FakeDriver()
    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            OneShotNav(),
            driver,
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=False),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 5.0}),
            statuses.append,
        )

    assert pulses == []
    assert driver.disengaged >= 1
    assert "not engaged" in statuses[-1]["guardrail_reasons"]


def test_run_manual_jog_bypasses_auto_engage(monkeypatch):
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda _driver, sign, duration_s: pulses.append((sign, duration_s)))
    manual = ManualState()
    manual.on_message(None, None, make_msg("openhelm/autopilot/vessel/manual", {"direction": "starboard"}))

    statuses = []
    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            OneShotNav(),
            FakeDriver(),
            SimpleNamespace(current=None),
            SimpleNamespace(engaged=False),
            manual,
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 5.0}),
            statuses.append,
        )

    assert pulses == [(1, pytest.approx(0.1))]
    assert statuses[-1]["manual_direction"] == "starboard"


def test_direct_http_server_accepts_publish_and_serves_status(monkeypatch):
    monkeypatch.setattr(autopilot, "mqtt", None)
    nav = LiveNav()
    engage = EngageState()
    route_state = RouteState()
    manual = ManualState()
    status = StatusState()
    status.set({"engaged": False})
    server = autopilot.DirectControlServer(0, nav, engage, route_state, manual, autopilot.PulseModeState(), status, host="127.0.0.1")
    server.start()
    try:
        base = f"http://127.0.0.1:{server._server.server_port}"
        body = json.dumps({"topic": autopilot.ENGAGE_TOPIC, "payload": {"engaged": True}}).encode("utf-8")
        req = request.Request(
            f"{base}/autopilot/publish",
            data=body,
            headers={"content-type": "application/json"},
            method="POST",
        )
        with request.urlopen(req, timeout=2) as resp:
            assert resp.status == 200
        with request.urlopen(f"{base}/autopilot/status", timeout=2) as resp:
            st = json.loads(resp.read())
        assert st["engaged"] is False
        # nav has never received a fix, so the live-fix block reports no position
        # but the keys are always present for consumers.
        assert st["lat"] is None and st["lon"] is None
        assert st["cog"] is None and st["sog"] is None
        assert st["gps_source"] is None
        assert st["fix_age_s"] is None
    finally:
        server.stop()
    assert engage.engaged is True


def test_direct_http_server_rejects_invalid_route(monkeypatch):
    monkeypatch.setattr(autopilot, "mqtt", None)
    server = autopilot.DirectControlServer(
        0,
        LiveNav(),
        EngageState(),
        RouteState(),
        ManualState(),
        autopilot.PulseModeState(),
        StatusState(),
        host="127.0.0.1",
    )
    server.start()
    try:
        body = json.dumps({"topic": autopilot.ROUTE_TOPIC, "payload": {"waypoints": [[0, 0]]}}).encode("utf-8")
        req = request.Request(
            f"http://127.0.0.1:{server._server.server_port}/autopilot/publish",
            data=body,
            headers={"content-type": "application/json"},
            method="POST",
        )
        with pytest.raises(error.HTTPError) as exc_info:
            request.urlopen(req, timeout=2)
        assert exc_info.value.code == 400
        assert json.loads(exc_info.value.read()) == {"error": "route rejected"}
    finally:
        server.stop()


def test_direct_http_server_invokes_publish_hook(monkeypatch):
    monkeypatch.setattr(autopilot, "mqtt", None)
    nav = LiveNav()
    engage = EngageState()
    route_state = RouteState()
    manual = ManualState()
    status = StatusState()
    events = []
    server = autopilot.DirectControlServer(
        0,
        nav,
        engage,
        route_state,
        manual,
        autopilot.PulseModeState(),
        status,
        host="127.0.0.1",
        on_publish=lambda topic, payload: events.append((topic, payload)),
    )
    server.start()
    try:
        body = json.dumps({"topic": autopilot.MANUAL_TOPIC, "payload": {"direction": "port"}}).encode("utf-8")
        req = request.Request(
            f"http://127.0.0.1:{server._server.server_port}/autopilot/publish",
            data=body,
            headers={"content-type": "application/json"},
            method="POST",
        )
        with request.urlopen(req, timeout=2) as resp:
            assert resp.status == 200
    finally:
        server.stop()
    assert events == [(autopilot.MANUAL_TOPIC, {"direction": "port"})]


class MultiShotNav:
    """Like OneShotNav but replays a fixed sequence of GPS fixes, one per tick."""

    def __init__(self, gps_sequence):
        self.gps_sequence = gps_sequence
        self.calls = 0
        self.heartbeat = Heartbeat(received_at=time.monotonic())
        self.gps_problem = None

    def wait_for_new_fix(self, timeout):
        self.calls += 1
        if self.calls > len(self.gps_sequence):
            raise KeyboardInterrupt
        return True

    def snapshot(self):
        return self.gps_sequence[self.calls - 1], self.heartbeat, 0.0, 0.5


def test_fixed_mode_drives_from_fixed_table_not_learned_model(monkeypatch):
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda _driver, sign, duration_s: pulses.append((sign, duration_s)))

    # error_deg will be 30 (route_bearing 30 - cog 0) -> fixed table gives 300ms.
    # Seed the learned model so it would predict a very different duration (600ms)
    # if it were (wrongly) the one driving.
    response_model = learn.ResponseModel(a=0.5, b=0.0, seed_a=0.5, seed_b=0.0)

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            OneShotNav(),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            lambda status: None,
            response_model=response_model,
            pulse_mode_state=SimpleNamespace(mode="fixed"),
            observation_tracker=learn.PulseObservationTracker(fixes_to_observe=1),
        )

    assert pulses == [(1, pytest.approx(0.3))]


def test_learned_mode_drives_from_learned_model_not_fixed_table(monkeypatch):
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda _driver, sign, duration_s: pulses.append((sign, duration_s)))

    # Same setup as above, but PULSE_MODE=learned should drive the 600ms
    # learned-model prediction instead of the fixed table's 300ms.
    response_model = learn.ResponseModel(a=0.5, b=0.0, seed_a=0.5, seed_b=0.0)

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            OneShotNav(),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            lambda status: None,
            response_model=response_model,
            pulse_mode_state=SimpleNamespace(mode="learned"),
            observation_tracker=learn.PulseObservationTracker(fixes_to_observe=1),
        )

    assert pulses == [(1, pytest.approx(0.6))]


def test_passive_learning_updates_coefficients_in_fixed_mode(monkeypatch):
    monkeypatch.setattr(autopilot, "drive_pulse", lambda *a: None)

    now = time.monotonic()
    gps_sequence = [
        GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=now),
        GpsFix(cog_deg=10.0, sog_kn=4.0, received_at=now),
    ]
    response_model = learn.ResponseModel(a=0.5, b=0.0, seed_a=0.5, seed_b=0.0)

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            MultiShotNav(gps_sequence),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            lambda status: None,
            response_model=response_model,
            pulse_mode_state=SimpleNamespace(mode="fixed"),  # learned model never drives, but should still learn passively
            observation_tracker=learn.PulseObservationTracker(fixes_to_observe=1),
        )

    assert response_model.a != 0.5


def test_run_switches_pulse_mode_mid_session_without_restart(monkeypatch):
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda _driver, sign, duration_s: pulses.append((sign, duration_s)))

    pulse_mode_state = autopilot.PulseModeState(initial_mode="fixed")

    class SwitchingNav(MultiShotNav):
        def wait_for_new_fix(self, timeout):
            if self.calls == 1:
                # Simulate OpenHelm publishing a live PULSE_MODE_TOPIC switch
                # between ticks, exactly like pulse_mode_state.on_message would.
                pulse_mode_state.mode = "learned"
            return super().wait_for_new_fix(timeout)

    now = time.monotonic()
    gps_sequence = [
        GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=now),
        GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=now),
    ]
    # No observation_tracker -- keeps this test focused on mode switching,
    # not entangled with passive learning shifting the coefficients mid-test.
    response_model = learn.ResponseModel(a=0.5, b=0.0, seed_a=0.5, seed_b=0.0)

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            SwitchingNav(gps_sequence),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            lambda status: None,
            response_model=response_model,
            pulse_mode_state=pulse_mode_state,
        )

    # First tick drives the fixed table's 300ms; second tick, after the live
    # switch, drives the learned model's 600ms -- same process, no restart.
    assert pulses == [(1, pytest.approx(0.3)), (1, pytest.approx(0.6))]


def test_build_status_includes_pulse_mode_and_learned_prediction():
    travel_budget = TravelBudget(hard_over_seconds={"positive": 10.0, "negative": 10.0})
    status = autopilot.build_status(
        True, [], travel_budget, error_deg=5.0,
        pulse_mode="learned", learned_duration_ms=42.0,  # build_status still takes a plain string
    )
    assert status["pulse_mode"] == "learned"
    assert status["learned_duration_ms"] == 42.0


def test_run_seeds_response_model_from_observed_cog_without_bench_calibration(monkeypatch):
    """The whole point of ResponseSeedBootstrap: come up on a boat whose rudder has
    never been measured. Fixed pulses drive, the bootstrap watches the COG they
    produce, and the learned model exists once enough pulses have landed."""
    monkeypatch.setattr(autopilot, "drive_pulse", lambda *_a: None)
    seeded = []

    class SeedingBootstrap(learn.ResponseSeedBootstrap):
        def observe(self, pulse_ms, observed_dps):
            result = super().observe(pulse_ms, observed_dps)
            if result is not None:
                seeded.append(result)
            return result

    now = time.monotonic()
    # Each tick: a big error so a pulse always fires, and COG walking steadily so
    # every observation reports a real, consistent response.
    gps_sequence = [GpsFix(cog_deg=float(i * 3), sog_kn=4.0, received_at=now) for i in range(12)]

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            MultiShotNav(gps_sequence),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            lambda status: None,
            response_model=None,  # no bench calibration available
            pulse_mode_state=SimpleNamespace(mode="fixed"),
            observation_tracker=learn.PulseObservationTracker(fixes_to_observe=1),
            seed_bootstrap=SeedingBootstrap(samples_needed=3),
        )

    assert seeded, "bootstrap never produced a seed from the observed COG response"
    seed_a, seed_b = seeded[0]
    assert seed_a > 0
    assert seed_b == 0.0


def test_learned_pulse_is_capped_against_a_pathological_model(monkeypatch):
    """predict_pulse_ms is desired_rate / response_rate, unbounded as the rate goes
    small. A near-zero response must not become a multi-second slam of the helm."""
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda _d, sign, duration_s: pulses.append((sign, duration_s)))

    now = time.monotonic()
    tiny = 1e-9  # response so weak the model wants an enormous pulse
    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            MultiShotNav([GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=now)]),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            lambda status: None,
            response_model=learn.ResponseModel(a=tiny, b=0.0, seed_a=tiny, seed_b=0.0),
            pulse_mode_state=SimpleNamespace(mode="learned"),
        )

    assert pulses
    assert pulses[0][1] == pytest.approx(autopilot.MAX_LEARNED_PULSE_MS / 1000.0)


def test_helm_slews_to_its_target_between_fixes_then_stops(monkeypatch):
    """The two-loop split. Only a new fix may move the *target* -- that is what killed
    the original bang-bang, where ten pulses fired off one stale error. But driving the
    helm toward that target must run every tick: gating it to 1 Hz capped the slew at
    300ms/second, so a few seconds of demanded rudder took half a minute to wind on and
    the boat swung 60deg past before it arrived.

    So: many pulses are correct here, but they must converge on the target and stop.
    """
    pulses = []
    monkeypatch.setattr(autopilot, "drive_pulse", lambda _d, sign, duration_s: pulses.append((sign, duration_s)))

    class SlowFixNav:
        """One new fix, then bare ticks -- a 10 Hz loop on 1 Hz GPS."""

        def __init__(self):
            self.calls = 0
            self.heartbeat = Heartbeat(received_at=time.monotonic())
            self.gps_problem = None
            self.gps = GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=time.monotonic())

        def wait_for_new_fix(self, timeout):
            self.calls += 1
            if self.calls > 40:
                raise KeyboardInterrupt
            return self.calls == 1  # only the first tick carries a new fix

        def snapshot(self):
            return self.gps, self.heartbeat, 0.0, 0.5

    travel_budget = TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0})
    config = SteeringConfig(rudder_s_per_error_deg=0.15)
    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            SlowFixNav(),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            config,
            travel_budget,
            lambda status: None,
            pulse_mode_state=SimpleNamespace(mode="fixed"),
        )

    # 30deg of error, no swing -> 30 * 0.15 = 4.5s of helm demanded.
    expected_target_s = 30.0 * config.rudder_s_per_error_deg
    assert travel_budget.position_s == pytest.approx(expected_target_s, abs=config.rudder_deadband_s)
    # It got there quickly rather than one pulse per fix...
    assert len(pulses) > 1
    # ...and then stopped, instead of pumping the helm onward for the remaining ticks.
    assert len(pulses) < 40, "helm must settle at its target, not keep driving"
    assert all(sign == 1 for sign, _ in pulses), "no chattering back and forth"


def test_status_keeps_publishing_between_fixes(monkeypatch):
    """Gating the drive must not gate the status card -- the phone would look frozen."""
    monkeypatch.setattr(autopilot, "drive_pulse", lambda *_a: None)
    published = []

    class SlowFixNav:
        def __init__(self):
            self.calls = 0
            self.heartbeat = Heartbeat(received_at=time.monotonic())
            self.gps_problem = None
            self.gps = GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=time.monotonic())

        def wait_for_new_fix(self, timeout):
            self.calls += 1
            if self.calls > 5:
                raise KeyboardInterrupt
            return self.calls == 1

        def snapshot(self):
            return self.gps, self.heartbeat, 0.0, 0.5

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            SlowFixNav(),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 100.0, "negative": 100.0}),
            published.append,
            pulse_mode_state=SimpleNamespace(mode="fixed"),
        )

    assert len(published) == 5, "status must publish every tick, not only on a new fix"
    assert all(s["error_deg"] is not None for s in published)


def test_run_builds_standing_helm_against_a_steady_error(monkeypatch):
    """End-to-end auto-trim: a persistent error, as a sail's weather helm gives, must
    accumulate standing rudder rather than settling at a permanent course offset."""
    monkeypatch.setattr(autopilot, "drive_pulse", lambda *_a: None)

    class SteadyErrorNav:
        """Course held 8deg off the leg bearing, fix after fix -- a steady disturbance."""

        def __init__(self):
            self.calls = 0
            self.heartbeat = Heartbeat(received_at=time.monotonic())
            self.gps_problem = None

        def wait_for_new_fix(self, timeout):
            self.calls += 1
            if self.calls > 300:
                raise KeyboardInterrupt
            return True

        def snapshot(self):
            gps = GpsFix(cog_deg=22.0, sog_kn=4.0, received_at=time.monotonic())
            return gps, self.heartbeat, 0.0, 0.5

    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            SteadyErrorNav(),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),  # leg bearing 30 -> steady +8deg error
            SimpleNamespace(engaged=True),
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 40.0, "negative": 40.0}),
            lambda status: None,
            pulse_mode_state=SimpleNamespace(mode="fixed"),
        )
    # Nothing to assert on the driver here -- the point is it ran the trim path without
    # blowing up; the accumulation behaviour itself is covered in test_control.py.


def test_run_clears_trim_when_it_stops_steering(monkeypatch):
    """Stale trim from one point of sail must not be waiting at the next engage."""
    monkeypatch.setattr(autopilot, "drive_pulse", lambda *_a: None)
    trims = []

    class WatchingTrim(autopilot.control.HelmTrim):
        def reset(self):
            trims.append("reset")
            super().reset()

    monkeypatch.setattr(autopilot.control, "HelmTrim", WatchingTrim)

    class DisengagedNav(MultiShotNav):
        pass

    now = time.monotonic()
    with pytest.raises(KeyboardInterrupt):
        autopilot.run(
            DisengagedNav([GpsFix(cog_deg=0.0, sog_kn=4.0, received_at=now)]),
            FakeDriver(),
            SimpleNamespace(current=FakeRoute()),
            SimpleNamespace(engaged=False),  # not engaged -> must reset trim
            ManualState(),
            {"positive_command": "starboard", "negative_command": "port"},
            GuardrailConfig(),
            SteeringConfig(),
            TravelBudget(hard_over_seconds={"positive": 40.0, "negative": 40.0}),
            lambda status: None,
            pulse_mode_state=SimpleNamespace(mode="fixed"),
        )

    assert trims, "trim must be reset while not steering"


# --- T-Deck NMEA over UDP, and source arbitration -------------------------

TDECK_RMC = "$GNRMC,151321.00,A,5829.32904,N,02151.85104,E,4.200,185.38,050826,,,A*45"
TDECK_RMC_STATIONARY = "$GNRMC,151323.00,A,5829.32904,N,02151.85104,E,0.000,,050826,,,A*63"


def _rmc(body):
    """Rebuilds a sentence with a correct checksum, so fixtures can't drift."""
    value = 0
    for char in body:
        value ^= ord(char)
    return f"${body}*{value:02X}"


def test_udp_listener_accepts_a_broadcast_fix():
    nav = LiveNav()
    listener = autopilot.NmeaUdpListener(nav)
    body = "GNRMC,151321.00,A,5829.32904,N,02151.85104,E,4.200,185.38,050826,,,A"
    listener.handle_datagram(_rmc(body).encode("ascii"))

    assert listener.fixes == 1
    assert nav.gps is not None
    assert abs(nav.gps.cog_deg - 185.38) < 1e-9
    assert abs(nav.gps.sog_kn - 4.2) < 1e-9
    assert abs(nav.lat - 58.4888173) < 1e-6
    assert nav.active_source == autopilot.SOURCE_TDECK


def test_udp_listener_refuses_to_invent_a_course_when_stationary():
    # A stopped boat has a position but no COG. Reporting 0.0 would read as due
    # north and could put the helm over at the dock.
    nav = LiveNav()
    listener = autopilot.NmeaUdpListener(nav)
    body = "GNRMC,151323.00,A,5829.32904,N,02151.85104,E,0.000,,050826,,,A"
    listener.handle_datagram(_rmc(body).encode("ascii"))

    assert nav.gps is None
    assert nav.gps_problem is not None
    assert listener.rejected == 1


def test_udp_listener_drops_corrupt_datagrams():
    nav = LiveNav()
    listener = autopilot.NmeaUdpListener(nav)
    listener.handle_datagram(b"$GNRMC,151321.00,A,5829.32904,N,02151.8510")  # truncated

    assert listener.fixes == 0
    assert nav.gps is None


def test_tdeck_fix_is_preferred_over_a_live_phone():
    # Both sources reporting: the phone must not interleave with the T-Deck, or
    # the loop sees COG jitter that is really two different receivers.
    nav = LiveNav()
    nav.apply_fix((58.0, 21.0, 100.0, 4.0), None, autopilot.SOURCE_TDECK, now=1000.0)
    accepted = nav.apply_fix((58.0, 21.0, 200.0, 4.0), None, autopilot.SOURCE_PHONE, now=1000.5)

    assert accepted is False
    assert nav.gps.cog_deg == 100.0
    assert nav.active_source == autopilot.SOURCE_TDECK


def test_phone_takes_over_once_the_tdeck_goes_quiet():
    nav = LiveNav()
    nav.apply_fix((58.0, 21.0, 100.0, 4.0), None, autopilot.SOURCE_TDECK, now=1000.0)
    still_early = nav.apply_fix((58.0, 21.0, 200.0, 4.0), None, autopilot.SOURCE_PHONE, now=1004.0)
    after_gap = nav.apply_fix((58.0, 21.0, 200.0, 4.0), None, autopilot.SOURCE_PHONE, now=1006.0)

    assert still_early is False
    assert after_gap is True
    assert nav.gps.cog_deg == 200.0
    assert nav.active_source == autopilot.SOURCE_PHONE


def test_tdeck_reclaims_immediately_when_it_comes_back():
    nav = LiveNav()
    nav.apply_fix((58.0, 21.0, 100.0, 4.0), None, autopilot.SOURCE_TDECK, now=1000.0)
    nav.apply_fix((58.0, 21.0, 200.0, 4.0), None, autopilot.SOURCE_PHONE, now=1006.0)
    reclaimed = nav.apply_fix((58.0, 21.0, 300.0, 4.0), None, autopilot.SOURCE_TDECK, now=1006.1)

    assert reclaimed is True
    assert nav.gps.cog_deg == 300.0
    assert nav.active_source == autopilot.SOURCE_TDECK


def test_phone_alone_still_works_when_no_tdeck_is_present():
    # The existing MQTT-only rig must be unaffected by the new preference.
    nav = LiveNav()
    nav.on_message(
        None, None,
        make_msg(GPS_TOPIC, {"lat": 57.7, "lon": 11.9, "sog": 4.2, "cog": 187, "ts": 1}),
    )
    assert nav.gps is not None
    assert nav.active_source == autopilot.SOURCE_PHONE


def test_nmea_udp_port_defaults_and_can_be_disabled(monkeypatch):
    monkeypatch.delenv("AUTOPILOT_NMEA_UDP_PORT", raising=False)
    assert autopilot.nmea_udp_port_from_env() == autopilot.DEFAULT_NMEA_UDP_PORT
    monkeypatch.setenv("AUTOPILOT_NMEA_UDP_PORT", "")
    assert autopilot.nmea_udp_port_from_env() is None
    monkeypatch.setenv("AUTOPILOT_NMEA_UDP_PORT", "10111")
    assert autopilot.nmea_udp_port_from_env() == 10111


def test_nav_status_fix_reports_the_live_fix_underway():
    nav = LiveNav()
    nav.apply_fix((57.7, 11.9, 187.0, 4.2), None, autopilot.SOURCE_TDECK, now=1000.0)

    st = autopilot.nav_status_fix(nav, now=1000.5)
    assert st["gps_source"] == autopilot.SOURCE_TDECK
    assert st["gps_problem"] is None
    assert st["lat"] == round(57.7, 6)
    assert st["lon"] == round(11.9, 6)
    assert st["cog"] == 187.0
    assert st["sog"] == 4.2
    assert st["fix_age_s"] == 0.5


def test_nav_status_fix_with_no_fix_is_all_null():
    st = autopilot.nav_status_fix(LiveNav(), now=1000.0)
    assert st["lat"] is None and st["lon"] is None
    assert st["cog"] is None and st["sog"] is None
    assert st["gps_source"] is None
    assert st["fix_age_s"] is None


def test_nav_status_fix_falls_back_to_last_position_when_stationary():
    # Underway first, so last_position is set; then the boat stops and the
    # steering-grade fix is cleared (cog unset). Status must still report
    # position so the phone/tab know where the boat is at the dock.
    nav = LiveNav()
    nav.apply_fix((57.7, 11.9, 187.0, 4.2), None, autopilot.SOURCE_TDECK, now=1000.0)
    nav.record_position(57.7001, 11.9001, None, 0.0, now=1010.0)
    nav.apply_fix(None, "cog unset (stationary)", autopilot.SOURCE_TDECK, now=1010.0)

    assert nav.gps is None  # not steerable
    st = autopilot.nav_status_fix(nav, now=1011.0)
    assert st["lat"] == round(57.7001, 6)
    assert st["lon"] == round(11.9001, 6)
    assert st["cog"] is None
    assert st["sog"] == 0.0
    assert st["fix_age_s"] == 1.0


def test_udp_listener_retains_position_when_stationary():
    nav = LiveNav()
    listener = autopilot.NmeaUdpListener(nav)
    body = "GNRMC,151323.00,A,5829.32904,N,02151.85104,E,0.000,,050826,,,A"
    listener.handle_datagram(_rmc(body).encode("ascii"))

    # Steering-grade fix is still refused (no invented course) ...
    assert nav.gps is None
    assert listener.rejected == 1
    # ... but the dockside position is kept for status consumers.
    assert nav.last_position is not None
    lat, lon, cog, sog, _received_at = nav.last_position
    assert abs(lat - 58.4888173) < 1e-6
    assert cog is None
    assert sog == 0.0


def test_status_endpoint_serves_the_live_fix(monkeypatch):
    monkeypatch.setattr(autopilot, "mqtt", None)
    nav = LiveNav()
    nav.apply_fix((57.7, 11.9, 187.0, 4.2), None, autopilot.SOURCE_TDECK, now=1000.0)
    status = StatusState()
    status.set({"engaged": False})
    server = autopilot.DirectControlServer(
        0, nav, EngageState(), RouteState(), ManualState(), autopilot.PulseModeState(), status, host="127.0.0.1"
    )
    server.start()
    try:
        base = f"http://127.0.0.1:{server._server.server_port}"
        with request.urlopen(f"{base}/autopilot/status", timeout=2) as resp:
            st = json.loads(resp.read())
    finally:
        server.stop()
    assert st["engaged"] is False
    assert st["lat"] == round(57.7, 6)
    assert st["cog"] == 187.0
    assert st["gps_source"] == autopilot.SOURCE_TDECK


def test_status_udp_broadcaster_pushes_json_to_subscribers():
    # The boat-LAN status feed: the Pi broadcasts, OpenHelm subscribes (no polling). Bind a
    # receiver on an ephemeral localhost port and confirm the status dict arrives verbatim.
    recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv.bind(("127.0.0.1", 0))
    port = recv.getsockname()[1]
    recv.settimeout(2)
    bc = autopilot.StatusUdpBroadcaster("127.0.0.1", port)
    try:
        bc.send({"engaged": True, "error_deg": 9.4})
        data, _addr = recv.recvfrom(4096)
    finally:
        bc.close()
        recv.close()
    assert json.loads(data.decode("utf-8")) == {"engaged": True, "error_deg": 9.4}
    assert bc.sent == 1


def test_status_udp_broadcaster_swallows_send_failures():
    # A dead/unroutable target must never raise into the control loop.
    bc = autopilot.StatusUdpBroadcaster("127.0.0.1", 1)  # port 1 is not broadcast-reachable here
    bc.send({"engaged": False})
    bc.close()
    assert bc.failed >= 0  # no raise is the contract


def test_status_udp_from_env_defaults_and_disables(monkeypatch):
    monkeypatch.delenv("AUTOPILOT_STATUS_UDP_PORT", raising=False)
    assert autopilot.status_udp_from_env() == (autopilot.DEFAULT_STATUS_UDP_BROADCAST, autopilot.DEFAULT_STATUS_UDP_PORT)
    monkeypatch.setenv("AUTOPILOT_STATUS_UDP_PORT", "")
    assert autopilot.status_udp_from_env() is None
    monkeypatch.setenv("AUTOPILOT_STATUS_UDP_PORT", "10112")
    monkeypatch.setenv("AUTOPILOT_STATUS_UDP_BROADCAST", "10.0.0.255")
    assert autopilot.status_udp_from_env() == ("10.0.0.255", 10112)
