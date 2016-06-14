#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from time import sleep
from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty

init_node('great_ardrones')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)
TAKEOFF_PUB = Publisher('/ardrone/takeoff', Empty, queue_size=1)


prev = 0
prev_time = 0


def main():
    """Main program"""

    TAKEOFF_PUB.publish(Empty())
    Subscriber("/ardrone/navdata", Navdata, handler)


def handler(navdata):
    """Handles the navdata input"""

    message = Twist()

    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200

    # PID Controlling

    error = 600 - navdata.altd
    time = navdata.header.stamp.secs + navdata.header.stamp.nsecs * 1e-9

    kp = 0.5
    ki = 0
    kd = 1

# Adding up
    p = kp * error
    i = ki # Not yet done
    d = kd * (error - prev) / (time - prev_time)
    message.linear.z = p + i + d

    print "###############################################"
    print navdata.altd
    print "###############################################"
    print "p = %f \t i = %f \t d = %f" % (p, i, d)
    print "###############################################"
    print message

    TWIST_PUB.publish(message)

    global prev
    prev = error
    global prev_time
    prev_time = time

if __name__ == '__main__':
    main()

    spin()
