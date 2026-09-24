import json

import pytest

from learn import (
    PulseObservationTracker,
    ResponseModel,
    ResponseSeedBootstrap,
    load_learned_calibration,
    log_pulse_outcome,
    outlier_weight,
    save_learned_calibration,
    seed_response_model,
)

OBSERVE_FIXES = 3


def test_predict_pulse_ms_scales_with_error_and_inverse_response():
    model = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    assert model.predict_pulse_ms(desired_turn_rate_dps=5.0, sog_kn=0.0) == pytest.approx(100.0)


def test_predict_pulse_ms_uses_sog_dependence():
    model = ResponseModel(a=0.05, b=0.01, seed_a=0.05, seed_b=0.01)
    assert model.predict_pulse_ms(desired_turn_rate_dps=5.0, sog_kn=5.0) == pytest.approx(50.0)


def test_update_clamps_a_to_half_to_double_seed():
    model = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    for _ in range(1000):
        model.update(predicted_dps=0.0, observed_dps=1000.0, pulse_ms=100, sog_kn=0.0, learning_rate=1.0)
    assert 0.5 * 0.05 <= model.a <= 2.0 * 0.05


def test_update_clamps_a_to_half_seed_on_negative_pressure():
    model = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    for _ in range(1000):
        model.update(predicted_dps=1000.0, observed_dps=0.0, pulse_ms=100, sog_kn=0.0, learning_rate=1.0)
    assert 0.5 * 0.05 <= model.a <= 2.0 * 0.05


def test_update_moves_a_toward_ground_truth():
    model = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    for _ in range(2000):
        predicted = model.predict_turn_rate_dps(pulse_ms=100, sog_kn=0.0)
        model.update(predicted_dps=predicted, observed_dps=6.0, pulse_ms=100, sog_kn=0.0, learning_rate=0.00002)
    assert model.a == pytest.approx(0.06, abs=0.005)


def test_update_leaves_b_pinned_when_seed_b_is_zero():
    model = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    model.update(predicted_dps=0.0, observed_dps=100.0, pulse_ms=100, sog_kn=5.0, learning_rate=1.0)
    assert model.b == 0.0


def test_outlier_weight_is_one_for_consistent_observation():
    history = [6.0, 6.1, 5.9, 6.0, 6.05]
    assert outlier_weight(observed_dps=6.0, recent_history=history) == pytest.approx(1.0)


def test_outlier_weight_is_reduced_for_wild_observation():
    history = [6.0, 6.1, 5.9, 6.0, 6.05]
    weight = outlier_weight(observed_dps=60.0, recent_history=history)
    assert 0.0 <= weight < 1.0


def test_outlier_weight_is_one_with_insufficient_history():
    assert outlier_weight(observed_dps=60.0, recent_history=[]) == pytest.approx(1.0)
    assert outlier_weight(observed_dps=60.0, recent_history=[6.0]) == pytest.approx(1.0)


def test_outlier_update_is_bounded_relative_to_normal_update():
    history = [6.0, 6.1, 5.9, 6.0, 6.05]
    baseline = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    baseline.update(predicted_dps=5.0, observed_dps=6.0, pulse_ms=100, sog_kn=0.0, learning_rate=0.001)
    normal_delta = abs(baseline.a - 0.05)

    outlier_model = ResponseModel(a=0.05, b=0.0, seed_a=0.05, seed_b=0.0)
    weight = outlier_weight(observed_dps=60.0, recent_history=history)
    outlier_model.update(predicted_dps=5.0, observed_dps=60.0, pulse_ms=100, sog_kn=0.0, learning_rate=0.001, weight=weight)
    outlier_delta = abs(outlier_model.a - 0.05)

    raw_error_ratio = 55.0 / 1.0
    assert outlier_delta / normal_delta < raw_error_ratio


def test_seed_response_model_averages_positive_and_negative_estimates():
    calibration = {
        "deg_per_second_estimate": {"positive": 18.0, "negative": -20.0},
        "pulse_samples": {
            "positive": [{"cumulative_ms": 600, "angle_deg": 18.0}],
            "negative": [{"cumulative_ms": 600, "angle_deg": -20.0}],
        },
    }
    seed_a, seed_b = seed_response_model(calibration)
    assert seed_a == pytest.approx((18.0 / 600.0 + 20.0 / 600.0) / 2.0, abs=1e-6)
    assert seed_b == 0.0


def test_seed_response_model_raises_without_deg_per_second_estimate():
    with pytest.raises(ValueError, match="deg_per_second_estimate"):
        seed_response_model({})


def test_save_then_load_round_trips_coefficients(tmp_path):
    path = tmp_path / "learned_calibration.json"
    model = ResponseModel(a=0.061, b=0.004, seed_a=0.05, seed_b=0.0)
    save_learned_calibration(path, model)

    loaded = json.loads(path.read_text())
    assert loaded["a"] == pytest.approx(0.061)
    assert loaded["b"] == pytest.approx(0.004)

    restored = load_learned_calibration(path, seed_a=0.05, seed_b=0.0)
    assert restored.a == pytest.approx(0.061)
    assert restored.b == pytest.approx(0.004)
    assert restored.seed_a == pytest.approx(0.05)


def test_load_learned_calibration_falls_back_to_seed_when_missing(tmp_path):
    path = tmp_path / "does_not_exist.json"
    restored = load_learned_calibration(path, seed_a=0.05, seed_b=0.01)
    assert restored.a == pytest.approx(0.05)
    assert restored.b == pytest.approx(0.01)


def test_tracker_returns_none_while_still_collecting_fixes():
    tracker = PulseObservationTracker(fixes_to_observe=OBSERVE_FIXES)
    tracker.pulse_fired(pulse_ms=100, sog_kn=4.0, cog_deg=90.0, fired_at_s=0.0, predicted_dps=6.0)
    assert tracker.observe_fix(cog_deg=91.0, at_s=1.0) is None
    assert tracker.observe_fix(cog_deg=92.0, at_s=2.0) is None


def test_tracker_returns_observation_after_enough_fixes():
    tracker = PulseObservationTracker(fixes_to_observe=OBSERVE_FIXES)
    tracker.pulse_fired(pulse_ms=100, sog_kn=4.0, cog_deg=90.0, fired_at_s=0.0, predicted_dps=6.0)
    tracker.observe_fix(cog_deg=91.0, at_s=1.0)
    tracker.observe_fix(cog_deg=92.0, at_s=2.0)
    result = tracker.observe_fix(cog_deg=93.0, at_s=3.0)
    assert result is not None
    assert result.pulse_ms == 100
    assert result.sog_kn == 4.0
    assert result.predicted_dps == 6.0
    assert result.observed_dps == pytest.approx(1.0)


def test_observe_fix_is_noop_with_no_pending_pulse():
    tracker = PulseObservationTracker(fixes_to_observe=OBSERVE_FIXES)
    assert tracker.observe_fix(cog_deg=91.0, at_s=1.0) is None


def test_tracker_handles_cog_wraparound():
    tracker = PulseObservationTracker(fixes_to_observe=1)
    tracker.pulse_fired(pulse_ms=100, sog_kn=4.0, cog_deg=350.0, fired_at_s=0.0, predicted_dps=6.0)
    result = tracker.observe_fix(cog_deg=10.0, at_s=1.0)
    assert result.observed_dps == pytest.approx(20.0)


def test_model_converges_toward_synthetic_ground_truth_with_sog_dependence():
    true_a, true_b = 0.05, 0.01
    model = ResponseModel(a=0.03, b=0.0, seed_a=0.05, seed_b=0.02)
    tracker = PulseObservationTracker(fixes_to_observe=1)

    cog = 0.0
    t = 0.0
    for step in range(5000):
        sog = 4.0 + (step % 5)
        pulse_ms = 100.0
        predicted = model.predict_turn_rate_dps(pulse_ms, sog)
        tracker.pulse_fired(pulse_ms, sog, cog_deg=cog, fired_at_s=t, predicted_dps=predicted)

        true_rate = true_a + true_b * sog
        cog += pulse_ms * true_rate
        t += 1.0

        outcome = tracker.observe_fix(cog_deg=cog % 360.0, at_s=t)
        model.update(
            predicted_dps=outcome.predicted_dps,
            observed_dps=outcome.observed_dps,
            pulse_ms=outcome.pulse_ms,
            sog_kn=outcome.sog_kn,
            learning_rate=0.000002,
        )

    assert model.a == pytest.approx(true_a, rel=0.1)
    assert model.b == pytest.approx(true_b, rel=0.2)


def test_log_pulse_outcome_appends_jsonl(tmp_path):
    path = tmp_path / "response_log.jsonl"
    log_pulse_outcome(path, pulse_ms=100, sog_kn=4.0, predicted_dps=6.0, observed_dps=5.5, drove="fixed")
    log_pulse_outcome(path, pulse_ms=200, sog_kn=5.0, predicted_dps=8.0, observed_dps=7.0, drove="learned")

    lines = path.read_text().strip().split("\n")
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first == {
        "pulse_ms": 100, "sog_kn": 4.0, "predicted_dps": 6.0, "observed_dps": 5.5, "drove": "fixed",
    }


# --- seeding from observed COG instead of a rudder-angle table ------------------------

def test_seed_bootstrap_returns_none_until_enough_samples():
    boot = ResponseSeedBootstrap(samples_needed=3)
    assert boot.observe(pulse_ms=100, observed_dps=5.0) is None
    assert boot.observe(pulse_ms=100, observed_dps=5.0) is None
    assert boot.samples_collected == 2


def test_seed_bootstrap_seeds_from_observed_response_rate():
    boot = ResponseSeedBootstrap(samples_needed=3)
    boot.observe(pulse_ms=100, observed_dps=5.0)  # 0.05 deg/s per ms
    boot.observe(pulse_ms=200, observed_dps=10.0)  # 0.05
    seed = boot.observe(pulse_ms=100, observed_dps=5.0)  # 0.05
    assert seed is not None
    seed_a, seed_b = seed
    assert seed_a == pytest.approx(0.05)
    assert seed_b == 0.0


def test_seed_bootstrap_uses_median_so_one_bad_fix_cannot_cage_the_model():
    # The clamp pins a to [0.5, 2.0] x seed_a forever, so a wild sample must not
    # decide the seed. Median of [0.05, 0.05, 5.0] is 0.05, mean would be ~1.7.
    boot = ResponseSeedBootstrap(samples_needed=3)
    boot.observe(pulse_ms=100, observed_dps=5.0)
    boot.observe(pulse_ms=100, observed_dps=5.0)
    seed_a, _ = boot.observe(pulse_ms=100, observed_dps=500.0)
    assert seed_a == pytest.approx(0.05)


def test_seed_bootstrap_uses_magnitude_so_port_turns_count():
    # A port pulse turns COG negative; it is just as good a response measurement.
    boot = ResponseSeedBootstrap(samples_needed=2)
    boot.observe(pulse_ms=100, observed_dps=-5.0)
    seed_a, _ = boot.observe(pulse_ms=100, observed_dps=5.0)
    assert seed_a == pytest.approx(0.05)


def test_seed_bootstrap_drops_samples_where_the_boat_did_not_answer():
    # A zero rate would collapse the clamp range to zero and pin the model dead.
    boot = ResponseSeedBootstrap(samples_needed=2)
    assert boot.observe(pulse_ms=100, observed_dps=0.0) is None
    assert boot.samples_collected == 0
    boot.observe(pulse_ms=100, observed_dps=4.0)
    seed_a, _ = boot.observe(pulse_ms=100, observed_dps=4.0)
    assert seed_a == pytest.approx(0.04)


def test_seed_bootstrap_ignores_nonpositive_pulse_width():
    boot = ResponseSeedBootstrap(samples_needed=1)
    assert boot.observe(pulse_ms=0, observed_dps=5.0) is None
    assert boot.samples_collected == 0


def test_seeded_model_is_usable_and_clamped_around_the_observed_seed():
    boot = ResponseSeedBootstrap(samples_needed=1)
    seed_a, seed_b = boot.observe(pulse_ms=100, observed_dps=5.0)
    model = ResponseModel(a=seed_a, b=seed_b, seed_a=seed_a, seed_b=seed_b)
    assert model.predict_turn_rate_dps(pulse_ms=100, sog_kn=4.0) == pytest.approx(5.0)
    # Drive it hard upward; the clamp must hold it at 2x the observed seed.
    for _ in range(500):
        model.update(predicted_dps=0.0, observed_dps=1000.0, pulse_ms=300,
                     sog_kn=4.0, learning_rate=0.01)
    assert model.a == pytest.approx(2.0 * seed_a)
