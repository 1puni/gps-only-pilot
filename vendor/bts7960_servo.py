#!/usr/bin/env python
#
# Vendored from Evita Stenqvist's local pypilot fork; see PROVENANCE.md.
# Driver code unchanged from commit 19dfa7b50058a0250165402e128ac4e5568ddcd7.
#
#   Copyright (C) 2026
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 3 of the
# License, or (at your option) any later version.

import os
import time


def _clamp(value, min_value, max_value):
    return min(max(value, min_value), max_value)


def _parse_bool(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.lower() in ['1', 'true', 'yes', 'on']


def _parse_float(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    return float(value)


def _parse_int(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    return int(value)


def _parse_duty(name, default):
    value = _parse_float(name, default)
    if value > 1:
        value /= 255.0
    return _clamp(value, 0, 1)


class BTS7960Config:
    def __init__(self):
        self.rpwm_pin = _parse_int('PYPILOT_BTS7960_RPWM', 18)
        self.lpwm_pin = _parse_int('PYPILOT_BTS7960_LPWM', 19)
        self.r_en_pin = _parse_int('PYPILOT_BTS7960_R_EN', 4)
        self.l_en_pin = _parse_int('PYPILOT_BTS7960_L_EN', 21)

        self.pwm_frequency = _parse_int('PYPILOT_BTS7960_PWM_FREQ', 1000)
        self.pwm_range = _parse_int('PYPILOT_BTS7960_PWM_RANGE', 255)

        # Accept either a fraction (0.35) or pigpio duty count (90).
        self.max_duty = _parse_duty('PYPILOT_BTS7960_MAX_DUTY', 90.0 / 255.0)
        self.min_duty = _parse_duty('PYPILOT_BTS7960_MIN_DUTY', 0)
        self.deadband = _parse_float('PYPILOT_BTS7960_DEADBAND', 0)
        self.slew_per_second = _parse_float('PYPILOT_BTS7960_SLEW', 0.50)
        self.reverse_pause = _parse_float('PYPILOT_BTS7960_REVERSE_PAUSE', 0.35)
        self.command_scale = _parse_float('PYPILOT_BTS7960_COMMAND_SCALE', 1)
        self.invert = _parse_bool('PYPILOT_BTS7960_INVERT', False)
        self.watchdog_seconds = _parse_float('PYPILOT_BTS7960_WATCHDOG', 1.0)

        self.min_duty = min(self.min_duty, self.max_duty)
        self.deadband = _clamp(self.deadband, 0, .95)
        self.slew_per_second = max(self.slew_per_second, .01)
        self.reverse_pause = max(self.reverse_pause, 0)
        self.watchdog_seconds = max(self.watchdog_seconds, .1)


class PigpioBackend:
    def __init__(self):
        import pigpio
        self.pigpio = pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('could not connect to pigpiod')

    def setup(self, config):
        for pin in [config.r_en_pin, config.l_en_pin]:
            self.pi.set_mode(pin, self.pigpio.OUTPUT)
            self.pi.write(pin, 0)
        for pin in [config.rpwm_pin, config.lpwm_pin]:
            self.pi.set_mode(pin, self.pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pin, config.pwm_frequency)
            self.pi.set_PWM_range(pin, config.pwm_range)
            self.pi.set_PWM_dutycycle(pin, 0)

    def pwm(self, pin, duty):
        self.pi.set_PWM_dutycycle(pin, duty)

    def write(self, pin, value):
        self.pi.write(pin, value)

    def close(self):
        self.pi.stop()


class BTS7960ServoDriver:
    controller_name = 'bts7960'
    has_current_feedback = False

    # Telemetry bit constants from servo.ServoTelemetry.  Keep them local to
    # avoid importing servo.py back into this driver while servo.py imports us.
    TELEMETRY_FLAGS = 1
    SERVO_FLAG_ENGAGED = 8
    SERVO_FLAG_DRIVER_TIMEOUT = 256 * 256 * 4

    def __init__(self, config=None, backend=None, clock=None):
        self.config = config or BTS7960Config()
        self.backend = backend or PigpioBackend()
        self.clock = clock or time.monotonic

        self.flags = 0
        self.current = False
        self.voltage = False
        self.controller_temp = False
        self.motor_temp = False
        self.rudder = False

        self._direction = 0
        self._duty = 0
        self._last_update = self.clock()
        self._last_command = self._last_update
        self._reverse_block_until = 0

        self.backend.setup(self.config)
        self.disengage()

    @classmethod
    def from_env(cls):
        return cls(BTS7960Config())

    def params(self, *args):
        # pypilot sends Arduino-controller limits on every command.  The
        # first BTS7960 version has no EEPROM/current/temperature feedback, so
        # these parameters remain pypilot-side policy for now.
        pass

    def _command_target(self, command):
        command = _clamp(float(command) * self.config.command_scale, -1, 1)
        if self.config.invert:
            command = -command
        magnitude = abs(command)
        if magnitude <= self.config.deadband:
            return 0, 0

        effective = (magnitude - self.config.deadband) / (1 - self.config.deadband)
        duty = self.config.min_duty + effective * (self.config.max_duty - self.config.min_duty)
        direction = 1 if command > 0 else -1
        return direction, _clamp(duty, 0, self.config.max_duty)

    def _slew(self, target, now):
        elapsed = max(0, now - self._last_update)
        step = self.config.slew_per_second * elapsed
        self._last_update = now

        if self._duty < target:
            self._duty = min(target, self._duty + step)
        elif self._duty > target:
            self._duty = max(target, self._duty - step)

        if self._duty < 0.0001:
            self._duty = 0
        return self._duty

    def _write_stop(self):
        self.backend.pwm(self.config.rpwm_pin, 0)
        self.backend.pwm(self.config.lpwm_pin, 0)
        self.backend.write(self.config.r_en_pin, 0)
        self.backend.write(self.config.l_en_pin, 0)
        self.flags &= ~self.SERVO_FLAG_ENGAGED

    def _write_drive(self, direction, duty):
        duty_count = int(round(_clamp(duty, 0, 1) * self.config.pwm_range))
        if duty_count <= 0 or direction == 0:
            self._write_stop()
            return

        if direction > 0:
            self.backend.pwm(self.config.lpwm_pin, 0)
            self.backend.pwm(self.config.rpwm_pin, duty_count)
        else:
            self.backend.pwm(self.config.rpwm_pin, 0)
            self.backend.pwm(self.config.lpwm_pin, duty_count)

        self.backend.write(self.config.r_en_pin, 1)
        self.backend.write(self.config.l_en_pin, 1)
        self.flags |= self.SERVO_FLAG_ENGAGED

    def command(self, command):
        now = self.clock()
        self.flags &= ~self.SERVO_FLAG_DRIVER_TIMEOUT
        target_direction, target_duty = self._command_target(command)

        if target_direction == 0:
            drive_direction = self._direction
            requested_duty = 0
        elif self._direction == 0:
            if now < self._reverse_block_until:
                drive_direction = 0
                requested_duty = 0
            else:
                drive_direction = target_direction
                requested_duty = target_duty
        elif self._direction != target_direction:
            drive_direction = self._direction
            requested_duty = 0
        else:
            drive_direction = target_direction
            requested_duty = target_duty

        previous_duty = self._duty
        duty = self._slew(requested_duty, now)
        if duty == 0:
            if previous_duty > 0:
                self._reverse_block_until = now + self.config.reverse_pause
            self._direction = 0
        else:
            self._direction = drive_direction

        self._last_command = now
        self._write_drive(self._direction, duty)

    def disengage(self):
        self._duty = 0
        self._direction = 0
        self._last_command = self.clock()
        self._write_stop()

    def poll(self):
        if self.flags & self.SERVO_FLAG_ENGAGED and \
           self.clock() - self._last_command > self.config.watchdog_seconds:
            self.flags |= self.SERVO_FLAG_DRIVER_TIMEOUT
            self.disengage()
        return self.TELEMETRY_FLAGS

    def fault(self):
        return False

    def reset(self):
        self.flags &= ~self.SERVO_FLAG_DRIVER_TIMEOUT
        self.disengage()

    def close(self):
        self.disengage()
        self.backend.close()
