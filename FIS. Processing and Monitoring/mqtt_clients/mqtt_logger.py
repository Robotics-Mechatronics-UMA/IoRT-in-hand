#!/usr/bin/env python3
"""
MQTT Logger
-----------
Subscribes to a set of MQTT topics and logs each received payload with a timestamp
into separate text files (one per topic).

Typical use case: debugging ROS→MQTT bridges or recording transformations
and joint states from a robotic system for later analysis.

Author : Juan Bravo-Arrabal
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import sys
from pathlib import Path
from typing import List

import paho.mqtt.client as mqtt

# --------------------------------------------------------------------------- #
#                              CONFIGURATION                                  #
# --------------------------------------------------------------------------- #

DEFAULT_TOPICS = [
    "aruco1/worldFrame",
    "aruco2/worldFrame",
    "aruco3/worldFrame",
    "aruco4/worldFrame",
    "bottomLegBandA1/worldFrame",
    "bottomLegBandA2/worldFrame",
    "upperLegBandA1/worldFrame",
    "upperLegBandA2/worldFrame",
    "end_effector",
    "jointsPR"
]

# --------------------------------------------------------------------------- #
#                             Argument parsing                                #
# --------------------------------------------------------------------------- #

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Subscribe to MQTT topics and log payloads with timestamps."
    )
    parser.add_argument(
        "-b", "--broker",
        default="192.168.1.100",
        help="MQTT broker IP or hostname (default: 192.168.1.100)",
    )
    parser.add_argument(
        "-p", "--port",
        type=int,
        default=1883,
        help="MQTT broker port (default: 1883)",
    )
    parser.add_argument(
        "-t", "--topics",
        nargs="+",
        default=DEFAULT_TOPICS,
        help="Space-separated list of MQTT topics to subscribe to.",
    )
    parser.add_argument(
        "-o", "--outdir",
        type=Path,
        default=Path.home() / "mqtt_logs",
        help="Directory where log files will be saved (default: ~/mqtt_logs).",
    )
    return parser.parse_args()

# --------------------------------------------------------------------------- #
#                              MQTT Callbacks                                 #
# --------------------------------------------------------------------------- #

def on_connect(client: mqtt.Client, userdata, flags, rc) -> None:
    """Called when the client connects to the MQTT broker."""
    if rc == 0:
        print("[MQTT] Connected successfully")
        client.connected_flag = True
        for topic in userdata["topics"]:
            client.subscribe(topic)
            print(f"[MQTT] Subscribed to topic: '{topic}'")
    else:
        print(f"[MQTT] Connection failed with code {rc}")
        sys.exit(1)


def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage) -> None:
    """Handles incoming messages by logging them with timestamps."""
    timestamp = dt.datetime.utcnow().isoformat()
    payload_str = msg.payload.decode("utf-8", errors="replace")
    log_line = f"{timestamp}  {payload_str}\n"

    logfile = userdata["outdir"] / f"{msg.topic.replace('/', '_')}.txt"
    with logfile.open("a", encoding="utf-8") as f:
        f.write(log_line)

    print(f"[LOG] {msg.topic}: {payload_str}")

# --------------------------------------------------------------------------- #
#                                    MAIN                                     #
# --------------------------------------------------------------------------- #

def main() -> None:
    args = parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)

    mqtt.Client.connected_flag = False  # type: ignore[attr-defined]
    client = mqtt.Client(client_id="logger")
    client.user_data_set({"topics": args.topics, "outdir": args.outdir})

    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(args.broker, args.port, keepalive=60)
    except Exception as exc:
        print(f"[ERROR] Could not connect to broker: {exc}")
        sys.exit(1)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[MQTT] Interrupted by user, shutting down...")
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()


"""
# Example usage:
python mqtt_logger.py -b IP_ADDRESS

* Change the IP address (-h) if your broker is hosted at a different location.

* Payloads with 16 elements represent 4×4 homogeneous transformation matrices.

* The end_effector topic expects a list in the format [px, py, pz, rx, ry, rz].

* The jointsPR topic simulates an array of 6 joint angles.

mosquitto_pub -p 1883 -t "aruco4/worldFrame" -m "[1.0, 0.0, 0.0, 0.25, 0.0, 1.0, 0.0, 0.35, 0.0, 0.0, 1.0, 0.45, 0.0, 0.0, 0.0, 1.0]"
mosquitto_pub -p 1883 -t "bottomLegBandA1/worldFrame" -m "[1.0, 0.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0, 1.0, 0.15, 0.0, 0.0, 0.0, 1.0]"
mosquitto_pub -p 1883 -t "upperLegBandA2/worldFrame" -m "[1.0, 0.0, 0.0, 0.08, 0.0, 1.0, 0.0, 0.13, 0.0, 0.0, 1.0, 0.18, 0.0, 0.0, 0.0, 1.0]"
mosquitto_pub -p 1883 -t "end_effector" -m "[0.4, 0.1, 0.3, 0.0, 0.0, 0.0]"
mosquitto_pub -p 1883 -t "jointsPR" -m "[0.1, 0.2, 0.3, -0.2, 0.5, -0.1]"

"""