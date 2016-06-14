#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

init_node('go_to_location')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)


target = [1, -1, 1]


def main():
    """Main program"""

    Subscriber("/vrpn_client_node/ardrone/pose", PoseStamped, handler)

def calc_p_control(kp, target, current):
    """Calculates p"""
    return kp * (target - current)


def handler(vrpn):
    """Handles the vrpn input"""

    position = [vrpn.pose.position.z, vrpn.pose.position.x,
            vrpn.pose.position.y]

    message = Twist()

    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200


    # Calc values

    kpx = 0.05
    kpy = 0.05
    kpz = 0.3
    kp_rotation = 1

    (r, p, y) = euler_from_quaternion([vrpn.pose.orientation.x,
        vrpn.pose.orientation.y, vrpn.pose.orientation.z,
        vrpn.pose.orientation.w])

    print "%f\t %f\t %f" % (r, p, y)

    message.linear.x = calc_p_control(kpx, target[0], position[0])
    message.linear.y = calc_p_control(kpy, target[1], position[1])
    message.linear.z = calc_p_control(kpz, target[2], position[2])
    message.angular.z = calc_p_control(kp_rotation, 0, p)



    print "###############################################"
    print "Position\t" + str(position)
    print "###############################################"
    print "Target\t" + str(target) + "\t" + str(p)
    print "###############################################"
    print message

    TWIST_PUB.publish(message)

if __name__ == '__main__':
    main()

    spin()
