from bts7960_servo import BTS7960Config, BTS7960ServoDriver


class FakeClock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeBackend:
    def __init__(self):
        self.pwm_values = {}
        self.pin_values = {}
        self.setup_config = None
        self.closed = False

    def setup(self, config):
        self.setup_config = config

    def pwm(self, pin, duty):
        self.pwm_values[pin] = duty

    def write(self, pin, value):
        self.pin_values[pin] = value

    def close(self):
        self.closed = True


def make_driver():
    config = BTS7960Config()
    config.max_duty = 90.0 / 255.0
    config.min_duty = 0
    config.deadband = 0
    config.slew_per_second = 10
    config.reverse_pause = .35
    config.watchdog_seconds = 1
    clock = FakeClock()
    backend = FakeBackend()
    driver = BTS7960ServoDriver(config, backend, clock)
    return driver, backend, clock, config


def test_disengage_holds_all_outputs_low():
    driver, backend, _, config = make_driver()

    driver.disengage()

    assert backend.pwm_values[config.rpwm_pin] == 0
    assert backend.pwm_values[config.lpwm_pin] == 0
    assert backend.pin_values[config.r_en_pin] == 0
    assert backend.pin_values[config.l_en_pin] == 0
    assert driver.flags == 0


def test_positive_command_drives_only_rpwm():
    driver, backend, clock, config = make_driver()

    clock.advance(.1)
    driver.command(1)

    assert backend.pwm_values[config.rpwm_pin] == 90
    assert backend.pwm_values[config.lpwm_pin] == 0
    assert backend.pin_values[config.r_en_pin] == 1
    assert backend.pin_values[config.l_en_pin] == 1
    assert driver.flags & driver.SERVO_FLAG_ENGAGED


def test_negative_command_drives_only_lpwm():
    driver, backend, clock, config = make_driver()

    clock.advance(.1)
    driver.command(-1)

    assert backend.pwm_values[config.rpwm_pin] == 0
    assert backend.pwm_values[config.lpwm_pin] == 90
    assert backend.pin_values[config.r_en_pin] == 1
    assert backend.pin_values[config.l_en_pin] == 1


def test_reverse_command_stops_before_driving_other_side():
    driver, backend, clock, config = make_driver()

    clock.advance(.1)
    driver.command(1)
    assert backend.pwm_values[config.rpwm_pin] == 90

    clock.advance(.1)
    driver.command(-1)
    assert backend.pwm_values[config.rpwm_pin] == 0
    assert backend.pwm_values[config.lpwm_pin] == 0
    assert backend.pin_values[config.r_en_pin] == 0
    assert backend.pin_values[config.l_en_pin] == 0

    clock.advance(.34)
    driver.command(-1)
    assert backend.pwm_values[config.lpwm_pin] == 0

    clock.advance(.02)
    driver.command(-1)
    assert backend.pwm_values[config.rpwm_pin] == 0
    assert 0 < backend.pwm_values[config.lpwm_pin] <= 90


def test_slew_limit_bounds_duty_change():
    driver, backend, clock, config = make_driver()
    config.slew_per_second = .25

    clock.advance(.2)
    driver.command(1)

    assert backend.pwm_values[config.rpwm_pin] == 13


def test_watchdog_disengages_stale_active_output():
    driver, backend, clock, config = make_driver()

    clock.advance(.1)
    driver.command(1)
    clock.advance(1.1)
    result = driver.poll()

    assert result & driver.TELEMETRY_FLAGS
    assert backend.pwm_values[config.rpwm_pin] == 0
    assert backend.pwm_values[config.lpwm_pin] == 0
    assert backend.pin_values[config.r_en_pin] == 0
    assert backend.pin_values[config.l_en_pin] == 0
    assert driver.flags & driver.SERVO_FLAG_DRIVER_TIMEOUT
