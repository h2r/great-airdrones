#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""ARDrone controlling class."""


from copy import deepcopy
import math
import time
import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped


class ARDrone:
    """ARDrone controlling class."""

    def __init__(self, name):
        """Constructor."""
        name = '/' + name + '/'

        self.__reset = rospy.Publisher(name + 'reset', Empty, queue_size=5)
        self.__land = rospy.Publisher(name + 'land', Empty, queue_size=5)
        self.__takeoff = rospy.Publisher(name + 'takeoff', Empty, queue_size=5)

        twistproxy = '/great_ardrones' + name + 'twist'
        self.__twist = rospy.Publisher(twistproxy, Twist, queue_size=5)

        self.__position = None
        self.__vrpnsequence = None
        self.__status = 'landed'
        self.__rotation = None
        self.__lastSeq = -1

        def getvrpndata(vrpndata):
            """Get vrpn position data and store into the class member."""
            point = vrpndata.pose.position
            angles = vrpndata.pose.orientation
            self.__rotation = [angles.x, -angles.z, angles.y]

            self.__position = [3.14*point.x, 3.14*-point.z, 3.14*point.y]
            self.__vrpnsequence = vrpndata.header.seq

        self.__getvrpndata = getvrpndata

        vrpntopic = '/vrpn_client_node' + name + 'pose'
        rospy.Subscriber(vrpntopic, PoseStamped, self.__getvrpndata)

    def takeoff(self):
        """Take the drone off."""
        self.__status = 'flying'
        self.__takeoff.publish(Empty())

    def land(self):
        """Land the drone."""
        self.settwist([0, 0, 0], [0, 0, 0])
        self.__land.publish(Empty())
        self.__status = 'landed'

    def reset(self):
        """Reset the drone."""
        self.settwist([0, 0, 0], [0, 0, 0])
        self.__reset.publish(Empty())
        self.__status = "landed"

    def settwist(self, linear, angular):
        """Publish twist."""
        message = Twist()

        message.linear.x = linear[0]
        message.linear.y = linear[1]
        message.linear.z = linear[2]

        message.angular.x = angular[0]
        message.angular.y = angular[1]
        message.angular.z = angular[2]

        self.__twist.publish(message)

    def getposition(self):
        """Return drone global position."""
        droneposition = deepcopy(self.__position)
        return droneposition

    def goto(self, wherepoint):
        """Make the drone going to <wherepoint>."""
        droneposition = deepcopy(self.__position)
        if droneposition is None:
            return "I can't get any vrpn data related to the drone"

        if self.__status == 'landed':
            self.takeoff()

        time_elapsed = time.clock()
        last_seq = self.__vrpnsequence

        # TODO: we need to run this inside thread
        for _ in range(1000000):
            if abs(time.clock() - time_elapsed) >= 0.4:
                if last_seq == self.__vrpnsequence:
                    self.__twist.publish(Twist())
                    self.__land.publish(Empty())
                    return "I just went out of the grid and landed"

                else:
                    last_seq = self.__vrpnsequence
                time_elapsed = time.clock()

            droneposition = deepcopy(self.__position)

            twistlinear = [0.0, 0.0, 0.0]
            twistangular = [0.0, 0.0, 0.0]

            dx = wherepoint[0] - droneposition[0]
            dy = wherepoint[1] - droneposition[1]
            dz = wherepoint[2] - droneposition[2]

            twistangular[2] = self.__rotation[1] - math.atan2(dx, dy)
            twistangular[2] = max(min(twistangular[1], +0.1), -0.1)

            twistlinear[2] = max(min(dz, 0.05), -0.05)

            if abs(twistangular[2]) < 0.005:
                twistlinear[0] = math.sqrt(dx*dx + dy*dy)
                twistlinear[0] = max(min(twistlinear[0], 0.05), -0.05)

                if max(twistlinear[0], twistlinear[2]) < 0.001:
                    break

            self.settwist(twistlinear, twistangular)

        return 'success'
