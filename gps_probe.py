#!/usr/bin/env python3
"""Stand-in for the Pi's direct-HTTP endpoint, to see what OpenHelm actually publishes.

Runs autopilot.py's real DirectControlServer, LiveNav and inspect_position_payload on a
laptop -- no Pi, no pigpiod, no calibration.json, no motor. Point OpenHelm's Autopilot tab
(Advanced -> Direct URL) at this machine and every payload the phone sends is printed here
with the same accept/reject verdict the Pi would reach.

It reports `engaged: true` in its status because OpenHelm's fast-GPS relay only publishes
while the controller confirms engagement (autopilotGps.ts's `if (!running) return`), so
without that the phone sends nothing and there is nothing to diagnose. It also echoes the
rejection reason back in `guardrail_reasons`, so the verdict shows up on the phone's own
status card as well as here.

    python3 gps_probe.py [--port 8765]
"""
import argparse
import json
import socket
import sys
import time
from collections import Counter

import autopilot


def local_addresses():
    """Best-effort list of addresses the phone could reach this machine on."""
    addresses = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 53))  # no packets sent; just picks the default route
        addresses.append(probe.getsockname()[0])
    except OSError:
        pass
    finally:
        probe.close()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = info[4][0]
            if not address.startswith("127.") and address not in addresses:
                addresses.append(address)
    except socket.gaierror:
        pass
    return addresses


class Probe:
    def __init__(self):
        self.topics = Counter()
        self.verdicts = Counter()
        self.last_problem = None
        self.started_at = time.monotonic()

    def status(self):
        reasons = [self.last_problem] if self.last_problem else []
        return {
            "engaged": True,
            "error_deg": None,
            "travel_budget_position_s": None,
            "guardrail_reasons": reasons,
            "leg_index": None,
            "distance_to_waypoint_m": None,
            "arrived": False,
            "manual_direction": None,
            "pulse_mode": None,
        }

    def on_publish(self, topic, payload):
        leaf = topic.rsplit("/", 1)[-1]
        self.topics[leaf] += 1
        stamp = f"{time.monotonic() - self.started_at:8.1f}s"

        if topic != autopilot.GPS_TOPIC:
            print(f"{stamp}  {leaf:<10} {json.dumps(payload)}", flush=True)
            return

        fix, problem = autopilot.inspect_position_payload(payload if isinstance(payload, dict) else {})
        if problem is None:
            self.verdicts["accepted"] += 1
            self.last_problem = None
            verdict = "OK"
        else:
            self.verdicts[problem] += 1
            self.last_problem = problem
            verdict = f"REJECTED -- {problem}"
        print(f"{stamp}  {leaf:<10} {json.dumps(payload)}  -> {verdict}", flush=True)

    def summary(self):
        lines = ["", "--- summary ---"]
        if not self.topics:
            lines.append("nothing received. The phone never reached this server.")
        for leaf, count in sorted(self.topics.items()):
            lines.append(f"{leaf}: {count} message(s)")
        for verdict, count in self.verdicts.most_common():
            lines.append(f"  gps {verdict}: {count}")
        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    probe = Probe()
    nav = autopilot.LiveNav()
    engage_state = autopilot.EngageState()
    route_state = autopilot.RouteState()
    manual_state = autopilot.ManualState()
    pulse_mode_state = autopilot.PulseModeState()
    status_state = autopilot.StatusState()
    status_state.set(probe.status())

    def on_publish(topic, payload):
        probe.on_publish(topic, payload)
        status_state.set(probe.status())

    server = autopilot.DirectControlServer(
        args.port, nav, engage_state, route_state, manual_state, pulse_mode_state,
        status_state, on_publish=on_publish,
    )
    server.start()

    print("Point OpenHelm's Autopilot tab -> Advanced -> Direct URL at one of:")
    for address in local_addresses():
        print(f"    http://{address}:{args.port}")
    print("\nReporting engaged=true, so the relay starts without pressing Engage.")
    print("Waiting for the phone... (Ctrl-C to stop)\n", flush=True)

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print(probe.summary())
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
