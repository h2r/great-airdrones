#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""ARDrone controlling class."""


from copy import deepcopy
import rospy
import time

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
            self.__position = [point.x, point.y, point.z]
            self.__vrpnsequence = vrpndata.header.seq

        self.__getvrpndata = getvrpndata

        vrpntopic = '/vrpn_client_node' + name + 'pose'
        rospy.Subscriber(vrpntopic, PoseStamped, self.__getvrpndata)

    def takeoff(self):
        """Take the drone off."""
        self.__takeoff.publish(Empty())
        self.__status = 'flying'

    def land(self):
        """Land the drone."""
        self.__land.publish(Empty())
        self.__status = 'landed'

    def reset(self):
        """Reset the drone."""
        self.__reset.publish(Empty())

    def settwist(self, linear, angular):
        """Publish twist."""
        message = Twist()

        message.linear.x = +linear[0]
        message.linear.y = -linear[2]
        message.linear.z = +linear[1]

        message.angular.x = angular[0]
        message.angular.y = angular[1]
        message.angular.z = angular[2]

        self.__twist.publish(message)

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
        for _ in range(10000):
            # if abs(time.clock() - time_elapsed) >= 0.01:
            #     if last_seq == self.__vrpnsequence:
            #         self.__twist.publish(Twist())
            #         self.__land.publish(Empty())
            #         return "I just went out of the grid and landed"
            #     else:
            #         self.__lastSeq = self.__vrpnsequence
            #     time_elapsed = time.clock()

            droneposition = deepcopy(self.__position)

            # TODO: consider orientation
            pointpairs = zip(wherepoint, droneposition)
            twistlinear = [0.01*(i - j) for i, j in pointpairs]

            if max([abs(u) for u in twistlinear]) < 0.0005:
                break

            twistangular = [0.0, 0.0, 0.0]

            twistlinear = [min(val, +0.001) for val in twistlinear]
            twistlinear = [max(val, -0.001) for val in twistlinear]

            self.settwist(twistlinear, twistangular)
