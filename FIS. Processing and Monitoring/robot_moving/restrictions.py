#!/usr/bin/env python3
"""
SafetyRestrictions node
-----------------------
Disables joystick command nodes when the end‑effector leaves the safe workspace.

• Subscribes : /end_effector   (std_msgs/String  -> "[x,y,z,rx,ry,rz]")
• Publishes  : /restriction_state (std_msgs/UInt8)
      0 = no restriction
      1 = Z‑axis disabled
      2 = XY‑plane disabled
Author : Juan Bravo‑Arrabal
"""

from __future__ import annotations
import ast
import logging
import sys
from typing import List

import rospy
from std_msgs.msg import String, UInt8
import rosnode

# --------------------------- Parameters ----------------------------------- #
Z_MIN  = rospy.get_param("/restrictions/z_min", 0.400)   # metres
Y_MIN  = rospy.get_param("/restrictions/y_min", -0.390)  # metres
NODE_Z = "/zAxis"    # node that sends Z velocity
NODE_XY = "/xyPlane" # node that sends XY velocity
# -------------------------------------------------------------------------- #

class SafetyRestrictions:
    """Monitors EE pose and (de)activates joystick nodes."""

    def __init__(self) -> None:
        self.killed_z  = False
        self.killed_xy = False

        self.state_pub = rospy.Publisher("/restriction_state", UInt8, queue_size=5)
        rospy.Subscriber("/end_effector", String, self.cb_pose)

    # ----------------------- Callbacks ------------------------------------ #
    def cb_pose(self, msg: String) -> None:
        try:
            pose = ast.literal_eval(msg.data)
            x, y, z = pose[:3]
        except (ValueError, SyntaxError, IndexError) as exc:
            rospy.logwarn("Invalid /end_effector message: %s", exc)
            return

        # ----- Z restriction ------------------------------------------------
        if z < Z_MIN and not self.killed_z:
            self.kill_node(NODE_Z)
            self.killed_z = True
            rospy.logwarn("Z BELOW %.3f m – node %s disabled", Z_MIN, NODE_Z)
        elif z >= Z_MIN and self.killed_z:
            self.killed_z = False
            rospy.loginfo("Z back in range – re‑enable %s manually if desired", NODE_Z)

        # ----- Y restriction ------------------------------------------------
        if y < Y_MIN and not self.killed_xy:
            self.kill_node(NODE_XY)
            self.killed_xy = True
            rospy.logwarn("Y BELOW %.3f m – node %s disabled", Y_MIN, NODE_XY)
        elif y >= Y_MIN and self.killed_xy:
            self.killed_xy = False
            rospy.loginfo("Y back in range – re‑enable %s manually if desired", NODE_XY)

        # Publish current restriction state
        state = 1 if self.killed_z else 2 if self.killed_xy else 0
        self.state_pub.publish(state)

    # ----------------------- Helpers -------------------------------------- #
    @staticmethod
    def kill_node(node_name: str) -> None:
        try:
            rosnode.kill_nodes([node_name])
        except rosnode.ROSNodeIOException as exc:
            rospy.logerr("Failed to kill %s: %s", node_name, exc)

# -------------------------------------------------------------------------- #
def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    rospy.init_node("safety_restrictions")
    SafetyRestrictions()
    rospy.loginfo("SafetyRestrictions node started")
    rospy.spin()

if __name__ == "__main__":
    main()
