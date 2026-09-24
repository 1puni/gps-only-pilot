# Source provenance

Original GPS-only pilot source: Evita Stenqvist and contributors, released under
GPL-3.0-or-later by the project owner. Existing copyright notices remain intact.
This export retains the current control implementations and tests, with public
configuration/docs and licensing changes recorded in the accompanying review
manifest. Private calibration, logs, handoffs and boot configuration are omitted.

`vendor/bts7960_servo.py` derives from Evita Stenqvist's local pypilot fork commit
`19dfa7b50058a0250165402e128ac4e5568ddcd7`, subject
`servo: add BTS7960 pigpio driver`, path `pypilot/bts7960_servo.py`, Git blob
`8c2862bdb0939d200715dde6e72054e174a9075a`. This identifies the local fork;
it does not claim origin in public upstream pypilot. The driver code is unchanged;
only its provenance comment differs. Its original GPL version 3 or later header
and the complete GPL version 3 license are retained. Driver tests are included.
