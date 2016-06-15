#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from time import sleep
from rospy import init_node, Subscriber, spin, Publisher, Time
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path

init_node('go_to_location')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)
MARKER_PUB = Publisher('/virtual/target', PoseStamped, queue_size=1)
POS_PUB = Publisher('/virtual/ardrone', PoseStamped, queue_size=1)
PATH_PUB = Publisher('/virtual/ardrone_path', Path, queue_size=1)


target = [0.955, -0.959, 1, 0]

prev_position = [0, 0, 0]
prev_time = 0

path = Path()
path.header.frame_id = "world"

num_observations = 0
sum_observations = [0, 0, 0]
square_observations = [0, 0, 0]

kpx = 0.0
kix = 0.0000000000
kdx = -1

kpy = 0.0
kiy = 0.0000000000
kdy = -1

kpz = 0.5
kiz = 0.000000000000
kdz = 0

kp_rotation = 0.5


# Variables for d calculations

first_d = [True, True, True]

a1 = [None, None, None]
a0 = [None, None, None]
t1 = [None, None, None]
t0 = [None, None, None]


def main():
    """Main program"""

    Subscriber("/vrpn_client_node/ardrone/pose", PoseStamped, handler,
            queue_size=1)

def calc_p_control(kp, target, current):
    """Calculates p"""
    return kp * (target - current)

def calc_i_control(ki, target, axis):
    prev_time = 0
    val = 0
    for pose in path.poses:
        time = pose.header.stamp.to_sec()
        if axis == "x":
            pos = pose.pose.position.x
        if axis == "y":
            pos = pose.pose.position.y
        if axis == "z":
            pos = pose.pose.position.z
        val += (time - prev_time) * (target - pos)
    return ki * val



def calc_d_control(kd, target, current, prev, time, prev_time, axis):
    """Calculates d"""
    coef1 = 0.95
    coef0 = 0.97

    # print str(a0) + "\tFIRST"

    if first_d[axis]:
        global a1
        a1[axis] = current
        global a0
        a0[axis] = current
        global t0
        t0[axis] = time
        global t1
        t1[axis] = time

        global first_d
        first_d[axis] = False

    else:
        global a1
        a1[axis] = coef1 * a1[axis] + (1.0 - coef1) * current

        global a0
        a0[axis] = coef0 * a0[axis] + (1.0 - coef0) * current

        global t1
        t1[axis] = coef1 * t1[axis] + (1.0 - coef1) * time

        global t0
        t0[axis] = coef0 * t0[axis] + (1.0 - coef0) * time


    # print str(a0) + "\tSECOND"
    # print str(time - t1) + "\t" + str(time - t0)
    # print sign + "\t" + str(abs(a1)) + "\t" + \
    numerator = a1[axis] - a0[axis]
    denominator = t1[axis] - t0[axis]
    # print str(numerator) + "\t" + str(denominator)
    return kd * numerator / denominator



def handler(vrpn):
    """Handles the vrpn input"""

    startTime = vrpn.header.stamp.to_sec()

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

    global num_observations
    num_observations += 1

    global sum_observations
    sum_observations[0] = target[0] - position[0]
    sum_observations[1] = target[1] - position[1]
    sum_observations[2] = target[2] - position[2]

    global square_observations
    square_observations[0] += (target[0] - position[0]) ** 2
    square_observations[1] += (target[1] - position[1]) ** 2
    square_observations[2] += (target[2] - position[2]) ** 2


# Updating Marker Position

    marker = PoseStamped()
    marker.header.frame_id = "world"
    marker.pose.position.x = target[0]
    marker.pose.position.y = target[1]
    marker.pose.position.z = target[2]
    (marker.pose.orientation.x, marker.pose.orientation.y,
            marker.pose.orientation.z, marker.pose.orientation.w) = \
    quaternion_from_euler(0, 0, target[3])

    MARKER_PUB.publish(marker)


    time = vrpn.header.stamp.to_sec()

    message = Twist()

    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200


    # Calc values



    px = calc_p_control(kpx, target[0], position[0])
    py = calc_p_control(kpy, target[1], position[1])
    pz = calc_p_control(kpz, target[2], position[2])

    p_rotation = calc_p_control(kp_rotation, target[3], p)

    dx = calc_d_control(kdx, target[0], position[0], prev_position[0], time,
            prev_time, 0)
    dy = calc_d_control(kdy, target[1], position[1], prev_position[1], time,
            prev_time, 1)
    dz = calc_d_control(kdz, target[2], position[2], prev_position[2], time,
            prev_time, 2)

    ix = calc_i_control(kix, target[0], "x")
    iy = calc_i_control(kiy, target[1], "y")
    iz = calc_i_control(kiz, target[2], "z")


    print "%f\t%f\t%f\t%f\t%f\t%f" % (px, py, pz, dx, dy, dz)
    # print "%f\t%f" % (dx, dy)

    message.linear.x = px + ix + dx
    message.linear.y = py + iy + dy
    message.linear.z = pz
    message.angular.z = p_rotation

    now = Time.now()
    # print now.to_sec() - startTime
    TWIST_PUB.publish(message)
    now = Time.now()
    # print now.to_sec() - startTime

    if len(path.poses) > 100000:
        path.poses.pop(0)
    path.poses.append(pos_pose)
    PATH_PUB.publish(path)

    global prev_position
    prev_position = position

    global prev_time
    prev_time = time


if __name__ == '__main__':
    main()

    sleep(30)

    mean = [sum_observations[0] / num_observations,
            sum_observations[1] / num_observations,
            sum_observations[2] / num_observations]

    square = [square_observations[0] / num_observations,
            square_observations[1] / num_observations,
            square_observations[2] / num_observations]

    # print [mean[0] ** 2 - square[0],
    #         mean[1] ** 2 - square[1],
    #         mean[2] ** 2 - square[2]]

    spin()
