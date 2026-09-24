import pytest

from control import (
    GuardrailConfig,
    GpsFix,
    Heartbeat,
    NavIntent,
    SteeringConfig,
    TravelBudget,
    command_sign_for_turn_right,
    decide_pulse,
    load_calibration,
    pulse_duration_ms,
    steer_enabled,
    steering_error,
    target_track,
    wrap180,
)


def test_wrap180_examples():
    assert wrap180(0) == 0
    assert wrap180(190) == -170
    assert wrap180(-190) == 170
    assert wrap180(180) == -180
    assert wrap180(-180) == -180


def test_target_track_subtracts_correction_for_positive_xte():
    nav = NavIntent(route_bearing_deg=90, cross_track_error_m=10, received_at=0)
    config = SteeringConfig(xte_correction_deg_per_meter=1, max_xte_correction_deg=30)
    assert target_track(nav, config) == pytest.approx(80)


def test_target_track_adds_correction_for_negative_xte():
    nav = NavIntent(route_bearing_deg=90, cross_track_error_m=-10, received_at=0)
    config = SteeringConfig(xte_correction_deg_per_meter=1, max_xte_correction_deg=30)
    assert target_track(nav, config) == pytest.approx(100)


def test_target_track_clamps_large_xte():
    nav = NavIntent(route_bearing_deg=90, cross_track_error_m=1000, received_at=0)
    config = SteeringConfig(xte_correction_deg_per_meter=1, max_xte_correction_deg=30)
    assert target_track(nav, config) == pytest.approx(60)


def test_steering_error_combines_target_track_and_cog():
    gps = GpsFix(cog_deg=80, sog_kn=5, received_at=0)
    nav = NavIntent(route_bearing_deg=90, cross_track_error_m=0, received_at=0)
    config = SteeringConfig()
    assert steering_error(gps, nav, config) == pytest.approx(10)


@pytest.mark.parametrize(
    "error_deg,expected_ms",
    [
        (0, None),
        (4.9, None),
        (5.0, 100),
        (9.9, 100),
        (10.0, 200),
        (19.9, 200),
        (20.0, 300),
        (90, 300),
        (-4.9, None),
        (-15, 200),
    ],
)
def test_pulse_duration_ms_table(error_deg, expected_ms):
    assert pulse_duration_ms(error_deg, deadband_deg=5.0) == expected_ms


def test_command_sign_for_turn_right_positive_is_starboard():
    assert command_sign_for_turn_right({"positive_command": "starboard"}) == 1


def test_command_sign_for_turn_right_positive_is_port():
    assert command_sign_for_turn_right({"positive_command": "port"}) == -1


def test_command_sign_for_turn_right_rejects_unknown():
    with pytest.raises(ValueError):
        command_sign_for_turn_right({"positive_command": "sideways"})


def test_decide_pulse_turn_right_uses_starboard_sign():
    config = SteeringConfig(deadband_deg=5)
    result = decide_pulse(15, {"positive_command": "starboard"}, config)
    assert result == (1, 200)


def test_decide_pulse_turn_left_uses_opposite_sign():
    config = SteeringConfig(deadband_deg=5)
    result = decide_pulse(-15, {"positive_command": "starboard"}, config)
    assert result == (-1, 200)


def test_decide_pulse_inside_deadband_is_none():
    config = SteeringConfig(deadband_deg=5)
    assert decide_pulse(2, {"positive_command": "starboard"}, config) is None


def test_travel_budget_clamps_to_hard_over():
    budget = TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 4.0})
    budget.apply_pulse(1, 10.0)
    assert budget.position_s == 5.0
    budget.apply_pulse(-1, 20.0)
    assert budget.position_s == -4.0


def test_travel_budget_at_limit():
    budget = TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 4.0}, position_s=5.0)
    assert budget.at_limit(1) is True
    assert budget.at_limit(-1) is False


def test_travel_budget_reports_remaining_time_per_direction():
    budget = TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 4.0}, position_s=4.75)
    assert budget.remaining_s(1) == pytest.approx(0.25)
    assert budget.remaining_s(-1) == pytest.approx(8.75)


def test_travel_budget_caps_requested_pulse_to_remaining_time():
    budget = TravelBudget(hard_over_seconds={"positive": 5.0, "negative": 4.0}, position_s=4.85)
    assert budget.allowed_pulse_s(1, 0.3) == pytest.approx(0.15)
    assert budget.allowed_pulse_s(-1, 0.3) == pytest.approx(0.3)


def test_steer_enabled_all_fresh():
    gps = GpsFix(cog_deg=0, sog_kn=5, received_at=100)
    nav = NavIntent(route_bearing_deg=0, cross_track_error_m=0, received_at=99)
    heartbeat = Heartbeat(received_at=100)
    enabled, reasons = steer_enabled(101, gps, nav, heartbeat, True, GuardrailConfig())
    assert enabled is True
    assert reasons == []


def test_steer_enabled_reports_all_failing_reasons():
    enabled, reasons = steer_enabled(1000, None, None, None, True, GuardrailConfig())
    assert enabled is False
    assert "gps fix stale" in reasons
    assert "openhelm heartbeat stale" in reasons
    assert "route data stale" in reasons


def test_steer_enabled_speed_too_low():
    gps = GpsFix(cog_deg=0, sog_kn=1.0, received_at=100)
    nav = NavIntent(route_bearing_deg=0, cross_track_error_m=0, received_at=100)
    heartbeat = Heartbeat(received_at=100)
    enabled, reasons = steer_enabled(100, gps, nav, heartbeat, True, GuardrailConfig(min_speed_kn=2.0))
    assert enabled is False
    assert reasons == ["speed over ground too low"]


def test_steer_enabled_default_min_speed_blocks_slow_headway():
    # 2 kn by default: nothing filters COG, so the gate stands in for a filter.
    gps = GpsFix(cog_deg=0, sog_kn=1.5, received_at=100)
    nav = NavIntent(route_bearing_deg=0, cross_track_error_m=0, received_at=100)
    heartbeat = Heartbeat(received_at=100)
    enabled, reasons = steer_enabled(100, gps, nav, heartbeat, True, GuardrailConfig())
    assert GuardrailConfig().min_speed_kn == 2.0
    assert enabled is False
    assert reasons == ["speed over ground too low"]


def test_steer_enabled_default_min_speed_allows_normal_headway():
    gps = GpsFix(cog_deg=0, sog_kn=2.5, received_at=100)
    nav = NavIntent(route_bearing_deg=0, cross_track_error_m=0, received_at=100)
    heartbeat = Heartbeat(received_at=100)
    enabled, reasons = steer_enabled(100, gps, nav, heartbeat, True, GuardrailConfig())
    assert enabled is True
    assert reasons == []


def test_steer_enabled_requires_engaged():
    gps = GpsFix(cog_deg=0, sog_kn=5, received_at=100)
    nav = NavIntent(route_bearing_deg=0, cross_track_error_m=0, received_at=99)
    heartbeat = Heartbeat(received_at=100)
    enabled, reasons = steer_enabled(101, gps, nav, heartbeat, False, GuardrailConfig())
    assert enabled is False
    assert reasons == ["not engaged"]


def test_steer_enabled_engaged_true_with_fresh_guardrails():
    gps = GpsFix(cog_deg=0, sog_kn=5, received_at=100)
    nav = NavIntent(route_bearing_deg=0, cross_track_error_m=0, received_at=99)
    heartbeat = Heartbeat(received_at=100)
    enabled, reasons = steer_enabled(101, gps, nav, heartbeat, True, GuardrailConfig())
    assert enabled is True
    assert reasons == []


def test_load_calibration_requires_direction_map_and_hard_over(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text('{"hard_over_seconds": {"positive": 1, "negative": 1}}')
    with pytest.raises(ValueError):
        load_calibration(path)


def test_load_calibration_rejects_missing_file(tmp_path):
    with pytest.raises(ValueError, match="not found -- run calibrate.py first"):
        load_calibration(tmp_path / "calibration.json")


def test_load_calibration_rejects_missing_hard_over_without_type_error(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text('{"direction_map": {"positive_command": "starboard", "negative_command": "port"}}')
    with pytest.raises(ValueError):
        load_calibration(path)


def test_load_calibration_rejects_non_opposite_direction_map(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text(
        '{"direction_map": {"positive_command": "starboard", "negative_command": "starboard"},'
        ' "hard_over_seconds": {"positive": 3.5, "negative": 4.0}}'
    )
    with pytest.raises(ValueError):
        load_calibration(path)


def test_load_calibration_rejects_nonpositive_hard_over(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text(
        '{"direction_map": {"positive_command": "starboard", "negative_command": "port"},'
        ' "hard_over_seconds": {"positive": 0, "negative": 4.0}}'
    )
    with pytest.raises(ValueError):
        load_calibration(path)


def test_load_calibration_reads_valid_file(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text(
        '{"direction_map": {"positive_command": "starboard", "negative_command": "port"},'
        ' "hard_over_seconds": {"positive": 3.5, "negative": 4.0}}'
    )
    direction_map, hard_over_seconds = load_calibration(path)
    assert direction_map["positive_command"] == "starboard"
    assert hard_over_seconds["positive"] == 3.5


# --- position-based rudder law (2026-08-04) -------------------------------------------

STARBOARD_MAP = {"positive_command": "starboard", "negative_command": "port"}
HARD_OVER = {"positive": 40.0, "negative": 40.0}


def _pulse(error_deg, turn_rate_dps, current_rudder_s, config=None):
    from control import decide_rudder_pulse
    return decide_rudder_pulse(
        error_deg, turn_rate_dps, current_rudder_s, HARD_OVER, STARBOARD_MAP,
        config or SteeringConfig(),
    )


def test_rudder_demand_is_proportional_to_error():
    from control import target_rudder_s
    config = SteeringConfig()
    small = target_rudder_s(5.0, 0.0, 10.0, STARBOARD_MAP, config)
    large = target_rudder_s(15.0, 0.0, 10.0, STARBOARD_MAP, config)
    assert 0 < small < large


def test_helm_backs_off_once_the_boat_is_already_swinging():
    """The whole point: if she has started turning the right way, stop adding helm."""
    from control import target_rudder_s
    config = SteeringConfig()
    standing_still = target_rudder_s(20.0, 0.0, 10.0, STARBOARD_MAP, config)
    already_turning = target_rudder_s(20.0, 5.0, 10.0, STARBOARD_MAP, config)
    assert already_turning < standing_still


def test_helm_reverses_when_swinging_too_fast_for_the_error():
    """Swinging hard toward the course with little error left -- take helm off, not on."""
    from control import target_rudder_s
    demand = target_rudder_s(2.0, 12.0, 10.0, STARBOARD_MAP, SteeringConfig())
    assert demand < 0


def test_rudder_unwinds_toward_centre_as_error_closes():
    # Helm is over, error nearly gone: the next move must be back toward centre.
    decision = _pulse(error_deg=0.5, turn_rate_dps=0.0, current_rudder_s=5.0)
    assert decision is not None
    assert decision[0] == -1


def test_rudder_demand_is_clamped_to_the_usable_range():
    # max_rudder_fraction is 1.0 by default now (Evita's call after sitting pinned at a
    # 30% limit while drifting off), so the demand clamp is the full hard-over and the
    # travel budget is what stops the motor driving past the stops. With a smaller
    # fraction configured, the clamp still binds.
    from control import target_rudder_s
    config = SteeringConfig(max_rudder_fraction=0.25)
    max_rudder_s = config.max_rudder_fraction * 40.0
    assert target_rudder_s(180.0, 0.0, max_rudder_s, STARBOARD_MAP, config) == pytest.approx(max_rudder_s)
    assert target_rudder_s(-180.0, 0.0, max_rudder_s, STARBOARD_MAP, config) == pytest.approx(-max_rudder_s)
    assert max_rudder_s < 40.0


def test_no_pulse_when_already_on_target_rudder():
    # Stops the helm chattering once settled.
    assert _pulse(error_deg=0.0, turn_rate_dps=0.0, current_rudder_s=0.0) is None


def test_single_pulse_is_bounded():
    config = SteeringConfig()
    _, duration_ms = _pulse(error_deg=90.0, turn_rate_dps=0.0, current_rudder_s=-10.0)
    assert duration_ms <= config.max_pulse_ms


def test_turn_rate_tracker_reports_signed_rate():
    from control import TurnRateTracker
    tracker = TurnRateTracker(smoothing=1.0)  # no smoothing, so the maths is visible
    tracker.update(cog_deg=100.0, at_s=0.0)
    assert tracker.update(cog_deg=105.0, at_s=1.0) == pytest.approx(5.0)
    assert tracker.update(cog_deg=100.0, at_s=2.0) == pytest.approx(-5.0)


def test_turn_rate_tracker_handles_cog_wraparound():
    from control import TurnRateTracker
    tracker = TurnRateTracker(smoothing=1.0)
    tracker.update(cog_deg=355.0, at_s=0.0)
    assert tracker.update(cog_deg=5.0, at_s=1.0) == pytest.approx(10.0)


def test_turn_rate_tracker_ignores_a_repeated_timestamp():
    from control import TurnRateTracker
    tracker = TurnRateTracker(smoothing=1.0)
    tracker.update(cog_deg=100.0, at_s=5.0)
    assert tracker.update(cog_deg=200.0, at_s=5.0) == pytest.approx(0.0)


def test_turn_rate_tracker_ignores_fixes_closer_than_the_minimum_interval():
    # The 2026-08-09 trial's 918 deg/s: a real 10deg COG step differentiated over the
    # ~11ms between two loop ticks draining a backlog of fixes.
    from control import TurnRateTracker
    tracker = TurnRateTracker(smoothing=1.0, min_dt_s=0.2)
    tracker.update(cog_deg=100.0, at_s=0.0)
    assert tracker.update(cog_deg=110.0, at_s=0.011) == pytest.approx(0.0)


def test_turn_rate_tracker_differentiates_from_the_last_trusted_sample():
    # Having refused the too-close fix above, the next one must span back to the last
    # sample we accepted -- not to the one we threw away.
    from control import TurnRateTracker
    tracker = TurnRateTracker(smoothing=1.0, min_dt_s=0.2)
    tracker.update(cog_deg=100.0, at_s=0.0)
    tracker.update(cog_deg=110.0, at_s=0.011)
    assert tracker.update(cog_deg=105.0, at_s=1.0) == pytest.approx(5.0)


def test_turn_rate_tracker_clamps_implausible_rates():
    # Backstop for any mechanism the min-interval guard doesn't catch, e.g. a bad fix.
    from control import TurnRateTracker
    tracker = TurnRateTracker(smoothing=1.0, max_rate_dps=30.0)
    tracker.update(cog_deg=0.0, at_s=0.0)
    assert tracker.update(cog_deg=170.0, at_s=0.25) == pytest.approx(30.0)


def test_delivered_travel_is_less_than_the_pulse_it_was_driven_for():
    # The driver ramps duty from zero every pulse, so the wall-clock width over-reads.
    from control import delivered_travel_s
    assert delivered_travel_s(0.8, slew_per_second=0.5, max_duty=0.353) < 0.8


def test_short_pulses_deliver_almost_nothing():
    # 100ms only reaches duty 0.05 of a 0.353 cap; it was credited a full 0.1s before.
    from control import delivered_travel_s
    assert delivered_travel_s(0.1, slew_per_second=0.5, max_duty=0.353) < 0.02


def test_delivered_travel_approaches_the_pulse_width_once_the_ramp_is_over():
    # A long pulse loses only the fixed ramp-up, so the two converge.
    from control import delivered_travel_s
    delivered = delivered_travel_s(10.0, slew_per_second=0.5, max_duty=0.353)
    assert delivered == pytest.approx(10.0 - 0.5 * (0.353 / 0.5))


def test_delivered_travel_is_never_negative():
    from control import delivered_travel_s
    assert delivered_travel_s(-1.0, slew_per_second=0.5, max_duty=0.353) == 0.0
    assert delivered_travel_s(0.5, slew_per_second=0.0, max_duty=0.353) == 0.0


def test_target_track_steers_at_the_mark_when_bearing_is_available():
    # 600m off the line used to saturate the +/-30deg cross-track clamp and hold a
    # heading 30deg off the leg; steering at the mark converges by construction.
    nav = NavIntent(
        route_bearing_deg=243, cross_track_error_m=-614, received_at=0,
        bearing_to_waypoint_deg=95,
    )
    assert target_track(nav, SteeringConfig()) == pytest.approx(95)


def test_target_track_falls_back_to_the_xte_law_without_a_waypoint_bearing():
    nav = NavIntent(route_bearing_deg=90, cross_track_error_m=10, received_at=0)
    config = SteeringConfig(xte_correction_deg_per_meter=1, max_xte_correction_deg=30)
    assert target_track(nav, config) == pytest.approx(80)


# --- auto-trim (integral) -------------------------------------------------------------

def _trim():
    from control import HelmTrim
    return HelmTrim()


def test_trim_accumulates_a_persistent_error():
    # A steady 10deg error, as a sail's weather helm produces, must build standing helm.
    trim = _trim()
    for _ in range(20):
        trim.update(error_deg=10.0, dt_s=1.0, max_trim_s=6.0)
    assert trim.trim_s > 0.5


def test_trim_takes_the_sign_of_the_error():
    up, down = _trim(), _trim()
    for _ in range(10):
        up.update(error_deg=10.0, dt_s=1.0, max_trim_s=6.0)
        down.update(error_deg=-10.0, dt_s=1.0, max_trim_s=6.0)
    assert up.trim_s > 0 and down.trim_s < 0


def test_trim_is_clamped():
    trim = _trim()
    for _ in range(10_000):
        trim.update(error_deg=90.0, dt_s=1.0, max_trim_s=6.0)
    assert trim.trim_s <= 6.0


def test_trim_does_not_wind_up_while_the_helm_is_saturated():
    """The guard that stops auto-trim becoming a 360: integrating through a turn the
    helm cannot influence, then releasing it all at once."""
    trim = _trim()
    for _ in range(50):
        trim.update(error_deg=10.0, dt_s=1.0, max_trim_s=6.0, saturated=True)
    assert trim.trim_s == 0.0


def test_trim_does_not_integrate_through_a_manoeuvre():
    # A large error is a course change, not a trim condition.
    trim = _trim()
    for _ in range(50):
        trim.update(error_deg=60.0, dt_s=1.0, max_trim_s=6.0)
    assert trim.trim_s == 0.0


def test_trim_ignores_a_nonpositive_interval():
    trim = _trim()
    trim.update(error_deg=10.0, dt_s=0.0, max_trim_s=6.0)
    assert trim.trim_s == 0.0


def test_trim_reset_drops_learned_standing_helm():
    trim = _trim()
    for _ in range(20):
        trim.update(error_deg=10.0, dt_s=1.0, max_trim_s=6.0)
    trim.reset()
    assert trim.trim_s == 0.0


def test_trim_shifts_the_commanded_rudder():
    from control import target_rudder_s
    config = SteeringConfig()
    without = target_rudder_s(5.0, 0.0, 10.0, STARBOARD_MAP, config, trim_s=0.0)
    with_trim = target_rudder_s(5.0, 0.0, 10.0, STARBOARD_MAP, config, trim_s=2.0)
    assert with_trim > without


def test_trim_lets_the_error_reach_zero_with_helm_still_on():
    """The whole point. With no error and no swing, P and D both demand zero helm --
    only trim can hold the standing rudder a sail needs."""
    from control import target_rudder_s
    config = SteeringConfig()
    assert target_rudder_s(0.0, 0.0, 10.0, STARBOARD_MAP, config, trim_s=0.0) == pytest.approx(0.0)
    assert target_rudder_s(0.0, 0.0, 10.0, STARBOARD_MAP, config, trim_s=3.0) == pytest.approx(3.0)
