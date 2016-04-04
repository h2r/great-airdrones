#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""ARDrone controlling class."""


import rospy
import copy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped


class ARDrone:
    """ARDrone controlling class."""

    def __init__(self, name):
        """Constructor."""
        self.__position = None
        self.__status = 'landed'

        self.__twist = rospy.Publisher('/great_ardrones/'+ name + '/twist', Twist)

        self.__reset = rospy.Publisher('/' + name + '/reset', Empty)
        self.__land = rospy.Publisher('/' + name + '/land', Empty)
        self.__takeoff = rospy.Publisher('/' + name + '/takeoff', Empty)

        def getvrpndata(vrpndata):
            p = vrpndata.pose.position
            self.__position = [p.x, p.y, p.z]

        vrpntopic = '/vrpn_client_node/' + name + '/pose'
        rospy.Subscriber(vrpntopic, PoseStamped, getvrpndata)

    def takeoff(self):
        """Take the drone off."""
        self.__takeoff.publish(Empty())
        self.__status = 'flying'

    def land(self):
        """Land the drone."""
        self.__land.publish(Empty())

    def reset(self):
        """Reset the drone."""
        self.__reset.publish(Empty())

    def __globalposition(self):
        """Get global position data."""
        if self.__position is not None:
            return copy.deepcopy(self.__position)
        else:
            print('Vrpn data is not available')
            quit()

    def settwist(self, twistlinear, twistangular):
        """Publish twist."""
        message = Twist()
        message.linear.x  = twistlinear[0]
        message.linear.y  = -1 * twistlinear[2]
        message.linear.z  = twistlinear[1]
        message.angular.x = twistangular[0]
        message.angular.y = twistangular[1]
        message.angular.z = twistangular[2]
        self.__twist.publish(message)

    def goto(self, wherepoint):
        """Go to <wherepoint>."""
        if self.__status == 'landed':
            self.takeoff()

# TODO: Account for turning
        droneposition = self.__globalposition()
        pointpairs = zip(wherepoint, droneposition)
        twistlinear = [0.01*(i - j) for i,j in pointpairs]
        twistangular = [0.0, 0.0, 0.0]
        while max([abs(u) for u in twistlinear]) > 0.001:
            droneposition = self.__globalposition()
            pointpairs = zip(wherepoint, droneposition)
            twistlinear = [0.01*(i - j) for i,j in pointpairs]

            twistlinear = [min(val,  0.001) for val in twistlinear]
            twistlinear = [max(val, -0.001) for val in twistlinear]
            print("test")
            self.settwist(twistlinear, twistangular)


