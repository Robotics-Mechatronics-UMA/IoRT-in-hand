#!/usr/bin/env python3
"""
IoRT‑in‑Hand node:
 ├─ Publishes force data  (/optoSensor)  + warning level  (/warning)
 ├─ Subscribes joystick   (/xyPlane, /zAxis) → UR speedL via RTDE
 ├─ Publishes joint state (/jointsPR)  + end‑effector pose (/end_effector)
 └─ Publishes EF‑to‑goal distance      (/distanceToGoal)

# Make the script executable
chmod +x iot_in_hand_node.py           

# Run the script
./iot_in_hand_node.py \             
    --robot-ip 192.168.88.10 \
    --sensor-ip 192.168.88.20

If you omit the flags, the script defaults to 192.168.1.105 (UR) and 192.168.1.11 (EtherDAQ), matching the previous behaviour.

Author: Juan Bravo‑Arrabal
"""


from __future__ import annotations
import argparse
import math
import threading
import time
from typing import Tuple

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8
import rtde_control
import rtde_receive
from etherdaq import EtherDAQ

# -----------------------  parameters (edit if needed)  -------------------- #

ROBOT_IP        = "192.168.1.105"
ETHERDAQ_IP     = "192.168.1.11"
FORCE_THRESHOLD = (6.0, 8.0)           # (yellow, red) in newtons
PUBLISH_RATE_HZ = 10                   # joints / pose publish rate
GOAL_POINT      = (-0.251, 0.127, 0.291)  # reference for distance calc

# ---------------------------------------------------------------------------

# --------------------------------------------------------------------------- #
# Argument parser – NEW                                                       #
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Unified IoRT‑in‑Hand ROS node (tele‑operation + sensing)."
    )
    parser.add_argument("--robot-ip",  default=ROBOT_IP,
                        help="IP address of the UR robot (RTDE).")
    parser.add_argument("--sensor-ip", default=ETHERDAQ_IP,
                        help="IP address of the EtherDAQ force sensor.")
    return parser.parse_args()
# --------------------------------------------------------------------------- #
# (classes EtherDAQWorker, URTeleopBridge, URStatePublisher remain unchanged) #
# --------------------------------------------------------------------------- #


class EtherDAQWorker(threading.Thread):
    """Continuously reads the EtherDAQ and publishes force + warning."""

    def __init__(self, sensor: EtherDAQ,
                 force_pub: rospy.Publisher,
                 warn_pub: rospy.Publisher) -> None:
        super().__init__(daemon=True)
        self.sensor = sensor
        self.force_pub = force_pub
        self.warn_pub = warn_pub
        self._stop = threading.Event()

    def run(self) -> None:
        rate = rospy.Rate(5)  # 5 Hz publish rate
        while not rospy.is_shutdown() and not self._stop.is_set():
            fx, fy, fz, tx, ty, tz = self.sensor.get_wrench()
            self.force_pub.publish(str(fz))           # publish Z‑force only
            if fz > FORCE_THRESHOLD[1]:
                self.warn_pub.publish(UInt8(0))       # red
            elif FORCE_THRESHOLD[0] <= fz <= FORCE_THRESHOLD[1]:
                self.warn_pub.publish(UInt8(1))       # yellow
            else:
                self.warn_pub.publish(UInt8(2))       # green
            rate.sleep()

    def stop(self) -> None:
        self._stop.set()


class URTeleopBridge:
    """Bridges joystick Twist messages to UR speed commands."""

    def __init__(self, rtde_ctrl: rtde_control.RTDEControlInterface) -> None:
        self.ctrl = rtde_ctrl
        rospy.Subscriber("/xyPlane", Twist, self.cb_xy)
        rospy.Subscriber("/zAxis",  Twist, self.cb_z)

    def cb_xy(self, msg: Twist) -> None:
        v = [msg.linear.x, msg.linear.y, 0.0, 0.0, 0.0, 0.0]
        self.ctrl.speedL(v)

    def cb_z(self, msg: Twist) -> None:
        v = [0.0, 0.0, msg.linear.z, 0.0, 0.0, 0.0]
        self.ctrl.speedL(v)


class URStatePublisher(threading.Thread):
    """Publishes joints, EF pose and distance to goal at fixed rate."""

    def __init__(self,
                 rtde_recv: rtde_receive.RTDEReceiveInterface,
                 joints_pub: rospy.Publisher,
                 pose_pub:   rospy.Publisher,
                 dist_pub:   rospy.Publisher,
                 rate_hz:    float = PUBLISH_RATE_HZ) -> None:
        super().__init__(daemon=True)
        self.recv = rtde_recv
        self.joints_pub = joints_pub
        self.pose_pub = pose_pub
        self.dist_pub = dist_pub
        self.rate_hz = rate_hz
        self._stop = threading.Event()

    def run(self) -> None:
        rad2deg = 180.0 / math.pi
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self._stop.is_set():
            q = [a * rad2deg for a in self.recv.getActualQ()]
            self.joints_pub.publish(str(q))

            tcp = self.recv.getActualTCPPose()
            pose_vec = [tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]]
            self.pose_pub.publish(str(pose_vec))

            dist = math.dist((tcp[0], tcp[1], tcp[2]), GOAL_POINT)
            self.dist_pub.publish(str(dist))

            rate.sleep()

    def stop(self) -> None:
        self._stop.set()

def main() -> None:
    args = parse_args()  # <--- parse runtime arguments

    rospy.init_node("iot_in_hand_unified", anonymous=False)

    # --- Publishers --------------------------------------------------------
    force_pub  = rospy.Publisher("/optoSensor", String, queue_size=10)
    warn_pub   = rospy.Publisher("/warning", UInt8, queue_size=10)
    joints_pub = rospy.Publisher("/jointsPR", String, queue_size=10)
    pose_pub   = rospy.Publisher("/end_effector", String, queue_size=10)
    dist_pub   = rospy.Publisher("/distanceToGoal", String, queue_size=10)

    # --- EtherDAQ setup ----------------------------------------------------
    ether = EtherDAQ(args.sensor_ip)  # <--- uses CLI argument
    ether.set_internal_filter_cutoff_frequency(15)
    ether.set_readout_rate(1000)
    ether.enable_internal_bias()
    ether.run_read_loop(background=True)

    force_thread = EtherDAQWorker(ether, force_pub, warn_pub)
    force_thread.start()

    # --- UR interfaces -----------------------------------------------------
    rtde_ctrl = rtde_control.RTDEControlInterface(args.robot_ip)  # <--- uses CLI argument
    rtde_recv = rtde_receive.RTDEReceiveInterface(args.robot_ip)

    # Joystick subscriber → UR control
    URTeleopBridge(rtde_ctrl)

    # State publisher thread
    state_thread = URStatePublisher(rtde_recv, joints_pub, pose_pub, dist_pub)
    state_thread.start()

    # Optional: print every /optoSensor message for debugging
    rospy.Subscriber("/optoSensor", String, lambda m: rospy.loginfo(f"Force Z: {m.data} N"))

    try:
        rospy.spin()
    finally:
        force_thread.stop()
        state_thread.stop()
        ether.stop_loop()
        rtde_ctrl.stopL()
        print("Unified IoRT‑in‑Hand node terminated.")


if __name__ == "__main__":
    main()



"""

Direction	Topic	Msg type	Purpose
Pub	/optoSensor	String	Z‑force in newtons (text).
Pub	/warning	UInt8	0 = red (>8 N), 1 = yellow (6–8 N), 2 = green (<6 N).
Pub	/jointsPR	String	Six joint angles in degrees.
Pub	/end_effector	String	[x,y,z,rx,ry,rz] pose vector.
Pub	/distanceToGoal	String	Scalar EF‑to‑goal distance (m).
Sub	/xyPlane	Twist	XY velocity commands from joystick.
Sub	/zAxis	Twist	Z velocity commands from joystick.


"""