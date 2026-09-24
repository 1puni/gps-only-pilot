# GPS-only pilot

A standalone experimental controller for a Raspberry Pi and BTS7960 drill-motor
steering rig. It follows waypoint routes using GPS course over ground, without
a compass, IMU or rudder-position sensor. Original source is GPL-3.0-or-later;
see LICENSE and THIRD-PARTY.md for retained third-party terms.

**Not cleared for hardware deployment or sea acceptance.** Real hard-over travel
must be measured before deployment. The earlier 60-second calibration was a mock,
not a measurement. No calibration file or operational sea log is distributed.
Software motor-travel estimates are not rudder feedback. GPS course is unreliable
at low speed and is not vessel heading. A physical emergency stop and competent
supervision remain necessary; tests use fake hardware.

## Test without hardware

Install Python 3.11 or newer and uv, then run:

```
uv run --locked python -B -m pytest -q -p no:cacheprovider
```

Tests use fake motor/GPIO backends and loopback HTTP fixtures. This command does
not start the controller, pigpiod, calibration or a motor service.

## Features and layout

- `control.py`: guardrails, adaptive course corrections, travel accounting.
- `learn.py`: turn-response learning and turn-rate estimation with correct
  millisecond differentiation; persisted learned parameters are local data.
- `autopilot.py`: starts disengaged; MQTT or direct HTTP route/control/GPS,
  NMEA UDP preference with phone fallback, manual holds, status broadcasting,
  off-course alarm and delivered motor-travel accounting.
- `route.py` and `nmea.py`: route geometry and checksum-checked NMEA parsing.
- `calibrate.py`: supervised direction, hard-over and pulse-table measurement.
- `vendor/`: driver and its fake-backend tests; exact origin in PROVENANCE.md.
- `gps_probe.py`: GPS transport probe; no motor driver.
- `tdeck_tiles.py`: converts user-supplied raster MBTiles to XYZ tiles for
  compatible T-Deck displays. No maps or map rights are supplied.

Read [SETUP.md](SETUP.md) before hardware operation and [DESIGN.md](DESIGN.md)
for limitations. The [drill-autopilot overview](OVERVIEW.md) describes the project.
OpenHelm is an optional client; no sibling repository is needed for this source.
