#!/usr/bin/env python3

from drone_maneuver import DroneManeuver

import rospy
import sys

if __name__ == "__main__":
    rospy.init_node("drone_maneuver")

    server = DroneManeuver()
    if not server.Initialize():
        rospy.logerr("Failed to initialize the move_server node.")
        sys.exit(1)

    rospy.spin()
