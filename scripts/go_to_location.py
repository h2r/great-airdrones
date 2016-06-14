#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

init_node('go_to_location')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)
MARKER_PUB = Publisher('/virtual/target', PoseStamped, queue_size=100)
POS_PUB = Publisher('/ardrone/vrpn_position', PoseStamped, queue_size=1)


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
    """Calculates d"""
    return ki * ((current - prev) / (time - prev_time))


def handler(vrpn):
    """Handles the vrpn input"""

    position = [vrpn.pose.position.z, vrpn.pose.position.x,
            vrpn.pose.position.y]

    (r, p, y) = euler_from_quaternion([vrpn.pose.orientation.x,
        vrpn.pose.orientation.y, vrpn.pose.orientation.z,
        vrpn.pose.orientation.w])

    pos_pose = PoseStamped()
    pos_pose.pose.position.x = position[0]
    pos_pose.pose.position.y = position[1]
    pos_pose.pose.position.z = position[2]
    pos_pose.pose.orientation = vrpn.pose.orientation
    pos_pose.header.stamp = vrpn.header.stamp
    pos_pose.header.frame_id = "world"

    POS_PUB.publish(pos_pose)

# Updating Marker Position

    marker = PoseStamped()
    marker.header.frame_id = "world"
    marker.pose.position.x = target[0]
    marker.pose.position.y = target[1]
    marker.pose.position.z = target[2]

    MARKER_PUB.publish(marker)


    time = vrpn.header.stamp.secs + vrpn.header.stamp.nsecs * 1e-9

    message = Twist()

    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200


    # Calc values

    kpx = 0.05
    kdx = 0.00001
    kpy = 0.05
    kdy = 0.00001
    kpz = 0.3
    kdz = 0.00001
    kp_rotation = 0.5


    px = calc_p_control(kpx, target[0], position[0])
    py = calc_p_control(kpy, target[1], position[1])
    pz = calc_p_control(kpz, target[2], position[2])

    p_rotation = calc_p_control(kp_rotation, 0, p)

    dx = calc_d_control(kdx, position[0], prev_position[0], time, prev_time)
    dy = calc_d_control(kdy, position[1], prev_position[1], time, prev_time)
    dz = calc_d_control(kdz, position[2], prev_position[2], time, prev_time)

    print "%f\t%f\t%f\t%f\t%f\t%f" % (px, py, pz, dx, dy, dz)

    message.linear.x = px + dx
    message.linear.y = py + dy
    message.linear.z = pz + dz
    message.angular.z = p_rotation

    TWIST_PUB.publish(message)

    global prev_position
    prev_position = position

    global prev_time
    prev_time = time

if __name__ == '__main__':
    main()

    spin()
