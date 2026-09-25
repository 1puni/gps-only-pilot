# Setup and calibration

The software requires Python >=3.11, paho-mqtt and the pigpio client. `uv.lock`
pins tested dependencies. Real GPIO operation additionally needs compatible
Raspberry Pi hardware and a separately installed pigpio daemon. No daemon,
automatic boot replacement or device credentials are included.

Before connecting motor power, verify wiring and driver pin/duty settings in
`vendor/bts7960_servo.py` against your rig. GPIO numbers are Broadcom numbering.
Provide an independent physical power cutoff. Do not deploy this candidate
before actual hard-over measurement: the historical 60 s value was only a mock.

`uv run python calibrate.py` is a hardware-moving command for a supervised
bench/dockside session, not a test. Its menu checks direction, measures travel
in both directions and records pulse/angle tables. Physically center the rudder
before each step. Stop before mechanical limits; the timeout is not an end-stop
sensor. Inspect every saved value in local `calibration.json` and complete both
directions. A partial file may exist after a partial session. Never reuse an
example or another vessel's travel time. Pulse estimates do not measure rudder
angle automatically.

Copy `autopilot.env.example` to a private `autopilot.env`, export the chosen
variables in your shell and configure calibration and optional route paths.
`uv run python autopilot.py` then starts the actual controller, initially
disengaged, and can move hardware after commands. It is deliberately not a test
or a deployment instruction for this unmeasured release.

For MQTT, supply a TLS WebSocket broker/account/password file and permissions
for `openhelm/autopilot/<AUTOPILOT_VESSEL_ID>/*`; the default ID is `vessel`.
Clients must use the same ID. No broker service or provisioning scripts are
required from this project. Direct HTTP, when explicitly enabled, exposes
`/autopilot/status` and `/autopilot/publish` on all interfaces without auth or
TLS. Use only an isolated trusted test network; it accepts motor commands.
Manual proxy mode requires a separately configured motor-web service and does
not make automatic control available without calibration.

NMEA UDP port 10110 is preferred; phone GPS becomes fallback when it goes stale.
The source used on the boat is a LilyGO T-Deck Plus running the experimental
[Meshtastic NMEA-broadcast fork](https://github.com/1puni/meshtastic-firmware),
which sends its u-blox M10's `$GNRMC`/`$GNGGA` at 3 Hz; any NMEA-0183-over-UDP
source on the same port works.
Set `AUTOPILOT_NMEA_UDP_PORT` empty to disable input. Status UDP can be disabled
with an empty `AUTOPILOT_STATUS_UDP_PORT` (as in the example); choose a broadcast
destination appropriate to your isolated network before enabling it.

The route example is invented and uses [latitude, longitude] pairs. It is only
for software exercises. Supply and verify your own route and clearance. Manual
jog intentionally bypasses GPS/route guardrails and has separate timeout/travel
checks. Neither mode supplies collision avoidance or measured rudder feedback.
