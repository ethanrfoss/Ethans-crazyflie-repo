#!/usr/bin/env python3

from crazyflie_msgs.msg import PositionVelocityStateStamped

import rospy
import std_msgs.msg
import std_srvs.srv
from std_srvs.srv import Empty

import numpy as np

class DroneManeuver(object):
    def __init__(self):
        self._initialized = False

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/drone_maneuver"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up a list of hard-coded reference points and maintain the current
        # index into the list.
        self._refs = np.array([0.0, -3.0, 2.0])

        self._initialized = True
        return True

    def LoadParameters(self):
        # Topics.
        if not rospy.has_param("~topics/ref"):
            return False
        self._ref_topic = rospy.get_param("~topics/ref")

        return True

    def RegisterCallbacks(self):
        # Publishers.
        self._ref_pub = rospy.Publisher(self._ref_topic, PositionVelocityStateStamped, queue_size=1)

        # Services.
        self._move_srv = rospy.Service("/maneuver", std_srvs.srv.Empty, self.maneuver)
        rospy.wait_for_service('takeoff')
        self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        rospy.wait_for_service('land')
        self._land = rospy.ServiceProxy('land', Empty)

        return True

    def maneuver(self, req):
        # Takeoff
        self._takeoff()

        # Wait 2 seconds
        rospy.sleep(2)

        # Move to New Position
        msg = PositionVelocityStateStamped();
        msg.header.stamp = rospy.Time.now()
        msg.state.x = self._refs[0]
        msg.state.y = self._refs[1]
        msg.state.z = self._refs[2]
        msg.state.x_dot = 0.0
        msg.state.y_dot = 0.0
        msg.state.z_dot = 0.0

        self._ref_pub.publish(msg)

        # Wait 10 Seconds to Stabilize
        rospy.sleep(10)

        # Land
        self._land()
