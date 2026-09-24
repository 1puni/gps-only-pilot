import calibrate


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class FakeDriver:
    def __init__(self):
        self.commands = []
        self.disengaged = False

    def command(self, sign):
        self.commands.append(sign)

    def disengage(self):
        self.disengaged = True


def test_drive_until_stopped_returns_when_enter_is_seen(monkeypatch):
    clock = FakeClock()
    waits = []

    def fake_wait_for_enter(timeout):
        waits.append(timeout)
        clock.now += timeout
        return len(waits) == 2

    monkeypatch.setattr(calibrate.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(calibrate, "wait_for_enter", fake_wait_for_enter)

    driver = FakeDriver()
    elapsed, hit_timeout = calibrate.drive_until_stopped(driver, 1, safety_timeout=10.0)

    assert hit_timeout is False
    assert elapsed == 0.2
    assert driver.commands == [1, 1]
    assert driver.disengaged is True


def test_drive_until_stopped_disengages_on_safety_timeout(monkeypatch):
    clock = FakeClock()

    def fake_wait_for_enter(timeout):
        clock.now += timeout
        return False

    monkeypatch.setattr(calibrate.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(calibrate, "wait_for_enter", fake_wait_for_enter)

    driver = FakeDriver()
    elapsed, hit_timeout = calibrate.drive_until_stopped(driver, -1, safety_timeout=0.25)

    assert hit_timeout is True
    assert elapsed == 0.25
    assert driver.commands == [-1, -1, -1]
    assert driver.disengaged is True
