"""Online-learned boat-response model, run alongside control.py's fixed
pulse table. See docs/superpowers/specs/2026-07-23-adaptive-boat-response-model-design.md.

No MQTT, no GPIO -- pure logic, same split as control.py.
"""
import json
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from control import wrap180

MIN_HISTORY_FOR_OUTLIER_CHECK = 2
OUTLIER_STDDEV_THRESHOLD = 3.0

# calibrate.py's step_pulse_table fires a 100+200+300ms sequence of pulses
# (sum of PULSE_DURATIONS_MS) and estimates an average deg/sec across that
# run -- this converts that rate back into a per-ms response constant.
CALIBRATION_PULSE_CUMULATIVE_MS = 600


@dataclass
class ResponseModel:
    """predicted_turn_rate_dps(pulse_ms, sog) = pulse_ms * (a + b * sog).

    seed_a/seed_b are the bench-calibrated starting point; a/b are clamped
    to [0.5, 2.0] times their seed at all times (see _clamp_coeff). A zero
    seed collapses that range to zero -- seed_b defaults to 0.0 ("no
    speed-dependence assumed until observed"), so b stays pinned at 0.0
    until a future spec revision addresses this; that's intentional, not
    a bug, per the spec's literal clamp rule.
    """
    a: float
    b: float
    seed_a: float
    seed_b: float

    def response_rate(self, sog_kn):
        return self.a + self.b * sog_kn

    def predict_pulse_ms(self, desired_turn_rate_dps, sog_kn):
        rate = self.response_rate(sog_kn)
        if rate <= 0:
            return 0.0
        return desired_turn_rate_dps / rate

    def predict_turn_rate_dps(self, pulse_ms, sog_kn):
        return pulse_ms * self.response_rate(sog_kn)

    def _clamp_coeff(self, value, seed):
        lo, hi = sorted((0.5 * seed, 2.0 * seed))
        return min(max(value, lo), hi)

    def update(self, predicted_dps, observed_dps, pulse_ms, sog_kn, learning_rate, weight=1.0):
        """One gradient step of (observed - predicted)^2 w.r.t. a, b, then clamp.

        d(error^2)/da = -2 * error * pulse_ms
        d(error^2)/db = -2 * error * pulse_ms * sog_kn
        The factor of 2 is absorbed into learning_rate (a tuning constant,
        not derived from theory). `weight` down-scales outlier observations
        -- see outlier_weight().
        """
        error = observed_dps - predicted_dps
        effective_rate = learning_rate * weight
        self.a = self._clamp_coeff(self.a + effective_rate * error * pulse_ms, self.seed_a)
        self.b = self._clamp_coeff(self.b + effective_rate * error * pulse_ms * sog_kn, self.seed_b)


def outlier_weight(observed_dps, recent_history):
    """1.0 for a normal observation, shrinking toward 0 the further
    observed_dps sits from recent_history's mean, in units of its stddev.

    Fewer than MIN_HISTORY_FOR_OUTLIER_CHECK samples: no basis to judge,
    weight 1.0. Callers must not include the new observation in
    recent_history before calling.
    """
    if len(recent_history) < MIN_HISTORY_FOR_OUTLIER_CHECK:
        return 1.0
    mean = statistics.mean(recent_history)
    stdev = statistics.pstdev(recent_history)
    if stdev == 0:
        return 1.0 if observed_dps == mean else 0.0
    deviations = abs(observed_dps - mean) / stdev
    if deviations <= OUTLIER_STDDEV_THRESHOLD:
        return 1.0
    return OUTLIER_STDDEV_THRESHOLD / deviations


def seed_response_model(calibration):
    """Derives (seed_a, seed_b) from calibrate.py's deg_per_second_estimate."""
    estimates = calibration.get("deg_per_second_estimate")
    if not isinstance(estimates, dict) or "positive" not in estimates or "negative" not in estimates:
        raise ValueError(
            "calibration is missing deg_per_second_estimate.positive/negative -- "
            "run calibrate.py's pulse/angle table steps first"
        )
    positive_rate = abs(estimates["positive"]) / CALIBRATION_PULSE_CUMULATIVE_MS
    negative_rate = abs(estimates["negative"]) / CALIBRATION_PULSE_CUMULATIVE_MS
    seed_a = (positive_rate + negative_rate) / 2.0
    seed_b = 0.0
    return seed_a, seed_b


@dataclass
class ResponseSeedBootstrap:
    """Derives (seed_a, seed_b) from the boat's own COG response to its first pulses.

    seed_response_model() needs calibrate.py's deg_per_second_estimate, which is a
    *rudder-angle* rate read off the quadrant by hand. This is the GPS-only route to
    the same number: fire the fixed pulse table, watch what the course actually did,
    and take the median of observed_dps / pulse_ms.

    Two reasons this is the better seed on this boat. It measures the quantity the
    model is actually fitted against -- ResponseModel.update() compares against
    observed_dps, which PulseObservationTracker computes from COG delta, not from
    rudder angle. And it needs no bench step, so the learned mode can come up on a
    boat that has never had its rudder measured.

    Median, not mean: ResponseModel clamps a to [0.5, 2.0] x seed_a forever, so one
    bad fix during bootstrap would cage the model permanently. Samples whose rate is
    zero or non-finite are dropped rather than counted -- a zero seed collapses that
    clamp range to zero and pins the model dead (see ResponseModel's docstring).
    """

    samples_needed: int
    _rates: list = field(default_factory=list, init=False)

    def observe(self, pulse_ms, observed_dps):
        """Feeds one pulse outcome in. Returns (seed_a, seed_b) once enough usable
        samples have landed, else None."""
        if pulse_ms <= 0:
            return None
        rate = abs(observed_dps) / pulse_ms
        if not math.isfinite(rate) or rate <= 0:
            return None  # the boat didn't answer the helm -- not a usable seed
        self._rates.append(rate)
        if len(self._rates) < self.samples_needed:
            return None
        # seed_b stays 0.0, matching seed_response_model: no speed-dependence
        # assumed until observed, and the clamp pins b there regardless.
        return statistics.median(self._rates), 0.0

    @property
    def samples_collected(self):
        return len(self._rates)


def save_learned_calibration(path, model):
    Path(path).write_text(json.dumps({"a": model.a, "b": model.b}, indent=2, sort_keys=True) + "\n")


def load_learned_calibration(path, seed_a, seed_b):
    path = Path(path)
    if not path.exists():
        return ResponseModel(a=seed_a, b=seed_b, seed_a=seed_a, seed_b=seed_b)
    data = json.loads(path.read_text())
    return ResponseModel(a=data["a"], b=data["b"], seed_a=seed_a, seed_b=seed_b)


def log_pulse_outcome(path, pulse_ms, sog_kn, predicted_dps, observed_dps, drove):
    record = {
        "pulse_ms": pulse_ms,
        "sog_kn": sog_kn,
        "predicted_dps": predicted_dps,
        "observed_dps": observed_dps,
        "drove": drove,
    }
    with open(path, "a") as fh:
        fh.write(json.dumps(record) + "\n")


@dataclass
class PulseOutcome:
    pulse_ms: float
    sog_kn: float
    predicted_dps: float
    observed_dps: float


@dataclass
class PulseObservationTracker:
    """Matches a fired pulse to the COG delta over the next fixes_to_observe
    GPS fixes, so learn.py can compute a (predicted, observed) pair for
    ResponseModel.update without autopilot.py needing to know the bookkeeping.
    """
    fixes_to_observe: int
    _pending: dict = field(default=None, init=False)
    _fixes_seen: int = field(default=0, init=False)

    def pulse_fired(self, pulse_ms, sog_kn, cog_deg, fired_at_s, predicted_dps):
        self._pending = {
            "pulse_ms": pulse_ms,
            "sog_kn": sog_kn,
            "start_cog_deg": cog_deg,
            "fired_at_s": fired_at_s,
            "predicted_dps": predicted_dps,
        }
        self._fixes_seen = 0

    def observe_fix(self, cog_deg, at_s):
        if self._pending is None:
            return None
        self._fixes_seen += 1
        if self._fixes_seen < self.fixes_to_observe:
            return None
        pending = self._pending
        self._pending = None
        elapsed_s = at_s - pending["fired_at_s"]
        if elapsed_s <= 0:
            return None
        cog_delta = wrap180(cog_deg - pending["start_cog_deg"])
        observed_dps = cog_delta / elapsed_s
        return PulseOutcome(
            pulse_ms=pending["pulse_ms"],
            sog_kn=pending["sog_kn"],
            predicted_dps=pending["predicted_dps"],
            observed_dps=observed_dps,
        )
