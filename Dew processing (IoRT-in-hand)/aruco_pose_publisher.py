#!/usr/bin/env python3
"""
aruco_pose_publisher.py
-----------------------
Publish ArUco pose (base → marker) to MQTT, using a ROS listener
for the end‑effector pose.

Author  : Juan Bravo‑Arrabal
License : MIT


# Default settings (camera 0, broker 192.168.1.100:1883, 30 fps)
python3 aruco_pose_publisher.py

# Custom broker and camera, lower FPS, larger median window
python3 aruco_pose_publisher.py \
        --broker-ip 10.0.0.5 --broker-port 1884 \
        --cam-id 1 --fps 15 --median 31

        
"""

from __future__ import annotations
import argparse
import logging
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Deque, Dict, Tuple

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import rospy
from cv2 import aruco
from std_msgs.msg import String

# --------------------------------------------------------------------------- #
# Argument parsing                                                            #
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ArUco pose publisher (ROS + MQTT).")
    p.add_argument("--broker-ip",   default="192.168.1.100", help="MQTT broker IP.")
    p.add_argument("--broker-port", type=int, default=1883,  help="MQTT broker port.")
    p.add_argument("--cam-id",      type=int, default=0,     help="OpenCV camera index.")
    p.add_argument("--fps",         type=float, default=30,  help="Max processing FPS.")
    p.add_argument("--median",      type=int, default=21,    help="Median filter window (odd).")
    p.add_argument("--matrix",      default="CameraMatrix.csv",   help="Intrinsic matrix file.")
    p.add_argument("--dist",        default="DistortionVector.csv", help="Distortion coeff file.")
    return p.parse_args()


# --------------------------------------------------------------------------- #
# Utility functions                                                           #
# --------------------------------------------------------------------------- #
def rodrigues_to_homogeneous(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert (rvec, tvec) from OpenCV to a 4×4 homogeneous matrix."""
    rot, _ = cv2.Rodrigues(rvec)
    return np.vstack((np.hstack((rot, tvec.reshape(3, 1))),
                      np.array([0, 0, 0, 1])))


def euler_xyz_to_homogeneous(tx, ty, tz, rx, ry, rz) -> np.ndarray:
    """Build 4×4 matrix from XYZ Euler angles (ROS convention, rad)."""
    cx, cy, cz = np.cos([rx, ry, rz])
    sx, sy, sz = np.sin([rx, ry, rz])

    rot_x = np.array([[1, 0, 0],
                      [0, cx, -sx],
                      [0, sx,  cx]])
    rot_y = np.array([[ cy, 0, sy],
                      [  0, 1,  0],
                      [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0],
                      [sz,  cz, 0],
                      [ 0,   0, 1]])
    rot = rot_z @ rot_y @ rot_x
    return np.vstack((np.hstack((rot, np.array([[tx], [ty], [tz]]))),
                      np.array([0, 0, 0, 1])))


# --------------------------------------------------------------------------- #
# Data classes                                                                #
# --------------------------------------------------------------------------- #
class EndEffectorListener:
    """Maintains the latest base → end‑effector transform from ROS."""

    def __init__(self) -> None:
        self._latest: Tuple[float, ...] | None = None
        rospy.Subscriber("/end_effector", String, self._callback)

    @property
    def transform(self) -> np.ndarray | None:
        return None if self._latest is None else euler_xyz_to_homogeneous(*self._latest)

    def _callback(self, msg: String) -> None:
        try:
            self._latest = tuple(float(v) for v in msg.data.strip("[]").split(","))
            if len(self._latest) != 6:
                raise ValueError
        except ValueError:
            rospy.logwarn("Invalid /end_effector message: %s", msg.data)


class MedianFilter:
    """Keeps a sliding window per marker ID and returns the median matrix."""

    def __init__(self, window: int) -> None:
        self._buffers: Dict[int, Deque[np.ndarray]] = defaultdict(
            lambda: deque(maxlen=window)
        )

    def add(self, marker_id: int, mat: np.ndarray) -> np.ndarray | None:
        buf = self._buffers[marker_id]
        buf.append(mat)
        if len(buf) < buf.maxlen:
            return None
        return np.median(np.stack(buf), axis=0)


# --------------------------------------------------------------------------- #
# Main publisher class                                                        #
# --------------------------------------------------------------------------- #
class PosePublisher:
    """Camera capture → ArUco detection → pose chain → MQTT publish."""

    TOPIC_MAP = {
        294: "aruco1/worldFrame",
        980: "aruco2/worldFrame",
        958: "aruco3/worldFrame",
        701: "aruco4/worldFrame",
        459: "upperLegBandA1/worldFrame",
        661: "upperLegBandA2/worldFrame",
        300: "bottomLegBandA1/worldFrame",
        674: "bottomLegBandA2/worldFrame",
    }

    E_TC = np.array([[1, 0, 0,  0],
                     [0, 1, 0, -0.10],
                     [0, 0, 1, -0.11],
                     [0, 0, 0,  1]], dtype=float)

    E_TU = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, -0.26],
                     [0, 0, 0, 1]], dtype=float)

    ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

    def __init__(self, args: argparse.Namespace) -> None:
        # MQTT
        self._client = mqtt.Client("aruco_publisher")
        self._client.on_connect = lambda *_: logging.info("MQTT connected")
        self._client.connect_async(args.broker_ip, args.broker_port)
        self._client.loop_start()

        # ROS
        rospy.init_node("aruco_pose_publisher", anonymous=True)
        self._ef_listener = EndEffectorListener()

        # Median filter
        self._median = MedianFilter(args.median)

        # Camera
        self._cap = cv2.VideoCapture(args.cam_id)
        if not self._cap.isOpened():
            logging.error("Cannot open camera %d", args.cam_id)
            sys.exit(1)

        # Intrinsics
        self._mtx = np.loadtxt(args.matrix, delimiter=",")
        self._dist = np.loadtxt(args.dist, delimiter=" ")

        # ArUco detector
        self._detector = aruco.ArucoDetector(self.ARUCO_DICT, aruco.DetectorParameters())

        # Rate control
        self._frame_delay = 1.0 / args.fps

    # --------------------------------------------------------------------- #
    def spin(self) -> None:
        last_time = 0.0
        try:
            while not rospy.is_shutdown():
                ok, frame = self._cap.read()
                if not ok:
                    continue

                # Throttle FPS
                now = time.time()
                if now - last_time < self._frame_delay:
                    continue
                last_time = now

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self._detector.detectMarkers(gray)
                if ids is None:
                    self._imshow(frame)
                    continue

                b_te = self._ef_listener.transform
                if b_te is None:
                    rospy.logdebug("Waiting for /end_effector…")
                    self._imshow(frame)
                    continue

                for c, marker_id in zip(corners, ids.flatten()):
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        c, 0.02, self._mtx, self._dist
                    )
                    c_t_a = rodrigues_to_homogeneous(rvec[0], tvec[0])
                    b_t_u = b_te @ self.E_TC @ c_t_a @ self.E_TU

                    mat = self._median.add(marker_id, b_t_u)
                    if mat is not None and marker_id in self.TOPIC_MAP:
                        self._client.publish(
                            self.TOPIC_MAP[marker_id],
                            payload=str(mat.flatten().tolist()),
                            qos=0,
                        )

                    aruco.drawDetectedMarkers(frame, [c])
                    aruco.drawAxis(frame, self._mtx, self._dist,
                                   rvec[0], tvec[0], 0.01)

                self._imshow(frame)
        finally:
            self._shutdown()

    # --------------------------------------------------------------------- #
    def _imshow(self, frame: np.ndarray) -> None:
        cv2.imshow("ArUco Pose", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User requested shutdown")

    def _shutdown(self) -> None:
        logging.info("Shutting down…")
        self._cap.release()
        cv2.destroyAllWindows()
        self._client.loop_stop()
        self._client.disconnect()


# --------------------------------------------------------------------------- #
# Entrypoint                                                                  #
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    PosePublisher(args).spin()
