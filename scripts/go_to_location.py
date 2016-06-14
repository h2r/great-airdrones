#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

init_node('go_to_location')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)


target = [1, -1, 1]

prev_position = [0, 0, 0]
prev_time = 0


def main():
    """Main program"""

    Subscriber("/vrpn_client_node/ardrone/pose", PoseStamped, handler)

def calc_p_control(kp, target, current):
    """Calculates p"""
    return kp * (target - current)

def calc_d_control(ki, current, prev, time, prev_time):
    """Calculates i"""
    return ki * ((current - prev) / (time - prev_time))


def handler(vrpn):
    """Handles the vrpn input"""

    position = [vrpn.pose.position.z, vrpn.pose.position.x,
            vrpn.pose.position.y]

    time = vrpn.header.stamp.secs + vrpn.header.stamp.nsecs * 1e-9

    message = Twist()

    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200


    # Calc values

    kpx = 0.05
    kix = 0.00
    kpy = 0.05
    kiy = 0.00
    kpz = 0.05
    kiz = 0.00
    kp_rotation = 1

    (r, p, y) = euler_from_quaternion([vrpn.pose.orientation.x,
        vrpn.pose.orientation.y, vrpn.pose.orientation.z,
        vrpn.pose.orientation.w])

    print "%f\t %f\t %f" % (r, p, y)

    message.linear.x = calc_p_control(kpx, target[0], position[0]) + \
        calc_d_control(kix, position[0], prev_position[0], time, prev_time)
    message.linear.y = calc_p_control(kpy, target[1], position[1]) + \
        calc_d_control(kiy, position[1], prev_position[1], time, prev_time)
    message.linear.z = calc_p_control(kpz, target[2], position[2]) + \
        calc_d_control(kiz, position[2], prev_position[2], time, prev_time)
    message.angular.z = calc_p_control(kp_rotation, 0, p)



    print "###############################################"
    print "Position\t" + str(position)
    print "###############################################"
    print "Target\t" + str(target) + "\t" + str(p)
    print "###############################################"
    print message

    TWIST_PUB.publish(message)

    global prev_position
    prev_position = position

    global prev_time
    prev_time = time

if __name__ == '__main__':
    main()

    spin()
