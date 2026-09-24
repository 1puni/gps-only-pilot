# A drill motor at the helm

An experimental autopilot pairs a drill-motor steering rig with a Raspberry Pi
and BTS7960 motor controller. It steers toward a waypoint route using GPS course
over ground, without a compass, IMU or rudder-position sensor.

OpenHelm can send the route and provide engage/disengage controls, status and
manual steering commands. The separate controller calculates course corrections
and drives the motor. It supports NMEA GPS input over the boat network, with a
phone-position fallback, and starts disengaged.

This is a supervised hardware experiment. GPS course over ground is not vessel
heading and is unreliable at low speed. Software checks stop automatic steering
when required inputs go stale, but estimated motor travel is not measured rudder
position. Calibration and an accessible physical kill switch remain necessary.
Manual steering commands have different input checks from automatic steering.
