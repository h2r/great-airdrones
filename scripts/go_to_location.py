#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from math import sin, cos, pi, atan2, sqrt
from time import sleep
from rospy import init_node, Subscriber, spin, Publisher, Time
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
quaternion_slerp, quaternion_inverse, quaternion_multiply
from nav_msgs.msg import Path

init_node('go_to_location')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)
MARKER_PUB = Publisher('/virtual/target', PoseStamped, queue_size=1)
POS_PUB = Publisher('/virtual/ardrone', PoseStamped, queue_size=1)
PATH_PUB = Publisher('/virtual/ardrone_path', Path, queue_size=1)


target = [-0.617, -0.404, 1, 0]
target_rotation = [0, 0, 0.5, 0.5]

prev_position = [0, 0, 0]
prev_time = 0

path = Path()
path.header.frame_id = "world"

num_observations = 0
sum_observations = [0, 0, 0]
square_observations = [0, 0, 0]

kpx = 0.1
kix = 0.0000000000
kdx = -0.25

kpy = 0.1
kiy = 0.0000000000
kdy = -0.25

kpz = 0.5
kiz = 0.000000000000
kdz = 0

kp_rotation = -0.1


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

def calc_offset_angle(current):
    """Calculates the offset angle from the x axis positiion"""

    x_axis = [1, 0, 0, 0]
    a = quaternion_multiply(x_axis, quaternion_inverse(current))
    rotation = quaternion_multiply(quaternion_multiply(a, x_axis), quaternion_inverse(a))
    angle = atan2(rotation[1], rotation[0])
    if abs(angle) < 0.1:
        angle = 0
    if angle < 0:
        angle += 2 * pi

    return angle


def calc_p_control_angle(beta, target, current):
    """Calculates p for the angle"""

    # print target
    # print current

    x_axis = [1, 0, 0, 0]
    a = quaternion_multiply(target, quaternion_inverse(current))
    rotation = quaternion_multiply(quaternion_multiply(a, x_axis), quaternion_inverse(a))
    # print rotation
    angle = atan2(rotation[1], rotation[0])
    if abs(angle) < 0.1:
        angle = 0
    if angle < 0:
        angle += 2 * pi
    after_p = calc_p_control(beta, 0, angle)
    # print str(angle/pi) + "\t" + str(after_p)
    return after_p


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
    coef1 = 0.9
    coef0 = 0.925

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


    numerator = a1[axis] - a0[axis]
    denominator = t1[axis] - t0[axis]
    return kd * numerator / denominator



def handler(vrpn):
    """Handles the vrpn input"""

    startTime = vrpn.header.stamp.to_sec()

    position = [vrpn.pose.position.z, vrpn.pose.position.x,
            vrpn.pose.position.y]

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
    marker.pose.orientation.x = target_rotation[0]
    marker.pose.orientation.y = target_rotation[1]
    marker.pose.orientation.z = target_rotation[2]
    marker.pose.orientation.w = target_rotation[3]

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

    unrotated = [0, 0, 0, 1]

    current_rotation = [vrpn.pose.orientation.z,
                -vrpn.pose.orientation.x, vrpn.pose.orientation.y,
                vrpn.pose.orientation.w]

    # target_rotation = [0.0086, -0.73, -0.031, 0.67]

    p_rotation = calc_p_control_angle(kp_rotation, target_rotation, current_rotation)

    dx = calc_d_control(kdx, target[0], position[0], prev_position[0], time,
            prev_time, 0)
    dy = calc_d_control(kdy, target[1], position[1], prev_position[1], time,
            prev_time, 1)
    dz = calc_d_control(kdz, target[2], position[2], prev_position[2], time,
            prev_time, 2)

    ix = calc_i_control(kix, target[0], "x")
    iy = calc_i_control(kiy, target[1], "y")
    iz = calc_i_control(kiz, target[2], "z")


    final_x = px + ix + dx
    final_y = py + iy + dy
    norm = sqrt(final_x ** 2 + final_y ** 2)

    global_goto_quat = [final_x / norm, final_y / norm, 0, 0]

    drone_goto_quat = quaternion_multiply(quaternion_multiply(current_rotation,
        global_goto_quat), quaternion_inverse(current_rotation))

    # new_norm = sqrt(drone_goto_quat[0] ** 2 + drone_goto_quat[1] ** 2)

    # Calculating quaternion transition

    offset_angle = calc_offset_angle(current_rotation)
    # print offset_angle / pi

    drone_goto = [cos(offset_angle) * final_x + sin(offset_angle) * final_y,
            cos(offset_angle) * final_y - sin(offset_angle) * final_x]

    new_norm = sqrt(drone_goto[0] ** 2 + drone_goto[1] ** 2)

    message.linear.x = drone_goto[0] / new_norm * norm
    message.linear.y = drone_goto[1] / new_norm * norm
    message.linear.z = pz
    message.angular.z = p_rotation * 0

    # print str(global_goto_quat) + "\t" + "GLOBAL"
    # print str(intermediate_quat) + "\t" + "INTER"
    # print current_rotation
    print "%f\t%f\t%f\t%f\t%f" % (final_x, final_y, message.linear.x,
            message.linear.y, message.angular.z)

    # print message.angular.z

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

    # calc_p_control_angle(0, [0, 0, 0, 1], [0, -0.7, 0, -0.7])

    spin()
