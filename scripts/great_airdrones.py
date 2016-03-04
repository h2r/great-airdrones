#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""Script to control airdrone from keyboard."""


import time
import roslib

roslib.load_manifest('great_airdrones')

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

if __name__ == "__main__":

    PUB = rospy.Publisher('cmd_vel', Twist)
    LAND_PUB = rospy.Publisher('/ardrone/land', Empty)
    RESET_PUB = rospy.Publisher('/ardrone/reset', Empty)
    TAKEOFF_PUB = rospy.Publisher('/ardrone/takeoff', Empty)

    rospy.init_node('test_flight')

    try:
        TWIST = Twist()

        ANGULAR, LINEAR = TWIST.angular, TWIST.linear
        LINEAR.x, LINEAR.y, LINEAR.z = 0, 0, 0
        ANGULAR.x, ANGULAR.y, ANGULAR.z = 0, 0, 0

        print("sleeping")
        time.sleep(5)

        print("takeoff")
        TAKEOFF_PUB.publish(Empty())
        for _ in range(0, 100):
            time.sleep(0.1)
            PUB.publish(TWIST)

        LAND_PUB.publish(Empty())
        for _ in range(0, 100):
            time.sleep(0.1)
            PUB.publish(TWIST)

    except ROSInternalException as exception:
        print(exception, repr(exception))

    except ServiceException as exception:
        print(exception, repr(exception))
