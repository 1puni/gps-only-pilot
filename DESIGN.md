# Control design

GPS course over ground and waypoint geometry provide track error and desired
course. The controller checks fix age, speed and route state before automatic
steering. NMEA receiver fixes take precedence over phone fixes so sources do not
alternate on every packet. Route math and motor decisions remain local.

The response model estimates turn response from observations, with time units
kept explicit. The retained source includes the millisecond turn-rate correction
and accounts for actual delivered motor travel instead of requested travel.
Software travel budgets estimate motion from motor timing; they cannot sense
slip, stalls, changing loads, mechanical stops or manual rudder movement.

Adaptive correction and off-course alerts assist supervised experiments; they
do not establish reliability at sea. GPS course does not distinguish heading
from leeway/current and loses usefulness at low speed. A compass/rudder-feedback
controller is a different feedback system and cannot be substituted by changing
gains alone. No certified navigation, obstacle avoidance or sea-trial acceptance
is claimed. Hard-over measurement and hardware validation remain outstanding.
