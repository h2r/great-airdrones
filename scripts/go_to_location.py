#!/usr/bin/env python2
# -*- coding: <utf-8> -*-

"""Hovering script"""

from math import sin, cos, pi, atan2
from time import sleep
from rospy import init_node, Subscriber, spin, Publisher, Time
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_inverse, quaternion_multiply
from nav_msgs.msg import Path
import numpy as np

init_node('go_to_location')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)
MARKER_PUB = Publisher('/virtual/target', PoseStamped, queue_size=1)
POS_PUB = Publisher('/virtual/ardrone', PoseStamped, queue_size=1)
PATH_PUB = Publisher('/virtual/ardrone_path', Path, queue_size=1)


target = [-0.617, -0.404, 1]
target_rotation = 0

prev_position = [0, 0, 0]
prev_time = 0

path = Path()
path.header.frame_id = "world"

num_observations = 0
sum_observations = [0, 0, 0]
square_observations = [0, 0, 0]

kpx = 0.3
kix = 0
kdx = -0.3

kpy = 0.3
kiy = 0
kdy = -0.3

kpz = 0.5
kiz = 0
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

    Subscriber("/vrpn_client_node/wand/pose", PoseStamped, wand_handler,
               queue_size=1)

    # Automated pd parameter finding
    log = open("p_logfile.txt", 'w')
    for i in xrange(1, 10):
        print "Going back"
        # Going back to starting location
        global kpx
        kpx = 0.3
        global kpy
        kpy = 0.3
        global kdx
        kdx = -0.3
        global kdy
        kdy = -0.3
        global target
        target = [0, 0, 1]
        sleep(10)

        # Testing new kdx, kdy
        kd = i / -10.0
        global kpx
        kpx = 0.6
        global kpy
        kpy = 0.6
        global kdx
        kdx = kd
        global kdy
        kdy = kd
        global target
        target = [-0.7, -0.7, 1]
        print "kd = %f\t i = %f" % (kd, i)

        # Wait for the drone to settle
        path.poses = []
        sleep(10)

        # Calculate variance over last 30 seconds
        x_terms = []
        y_terms = []
        for pose in path.poses:
            x_terms.append(pose.pose.position.x)
            y_terms.append(pose.pose.position.y)

        x_variance = np.var(np.array(x_terms))
        y_variance = np.var(np.array(y_terms))

        print "%f\t%f" % (x_variance, y_variance)

        # Writes kdx \t kdy \t variance
        log.write("%f\t%f\t%f\t%f\n" % (kd, kd, x_variance, y_variance))
    kdx = -0.3
    kdy = -0.3
    target = [0, 0, 1]
    log.close()
    print "done"


def global_to_drone_coordinates(mat, angle):
    """Converts from global to drone coordinates"""
    return [cos(angle) * mat[0] + sin(angle) * mat[1],
            cos(angle) * mat[1] - sin(angle) * mat[0], mat[2], mat[3]]


def calc_offset_angle(current):
    """Calculates the offset angle from the x axis positiion"""

    x_axis = [1, 0, 0, 0]
    a = quaternion_multiply(x_axis, quaternion_inverse(current))
    rotation = quaternion_multiply(quaternion_multiply(a, x_axis), quaternion_inverse(a))
    angle = atan2(rotation[1], rotation[0])
    # if angle < 0:
    #     angle += 2 * pi

    return angle


def calc_p_control_angle(beta, target, current):
    """Calculates p for the angle"""

    # x_axis = [1, 0, 0, 0]
    # a = quaternion_multiply(target, quaternion_inverse(current))
    # rotation = quaternion_multiply(quaternion_multiply(a, x_axis),
    #                             quaternion_inverse(a))
    # # print rotation
    # angle = atan2(rotation[1], rotation[0])
    # # if abs(angle) < 0.10:
    # #     angle = 0
    # if angle < 0:
    #     angle += 2 * pi
    # if angle < pi:
    #     angle = -angle
    # # elif angle > pi:
    # #     angle -= pi
    # print angle / pi
    angle = calc_offset_angle(current)
    after_p = calc_p_control(beta, target, angle)
    # print "%f\t%f\t%f" % (target - angle, after_p, angle)
    return after_p


def calc_p_control(kp, target, current):
    """Calculates p"""
    return kp * (target - current)

# def calc_i_control(ki, target, axis):
    # prev_time = 0
    # val = 0
    # for pose in path.poses:
    #     time = pose.header.stamp.to_sec()
    #     if axis == "x":
    #         pos = pose.pose.position.x
    #     if axis == "y":
    #         pos = pose.pose.position.y
    #     if axis == "z":
    #         pos = pose.pose.position.z
    #     val += (time - prev_time) * (target - pos)
    # return ki * val


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

    current_rotation = [vrpn.pose.orientation.z,
                -vrpn.pose.orientation.x, vrpn.pose.orientation.y,
                vrpn.pose.orientation.w]

    pos_pose = PoseStamped()
    pos_pose.pose.position.x = position[0]
    pos_pose.pose.position.y = position[1]
    pos_pose.pose.position.z = position[2]
    pos_pose.pose.orientation.x = current_rotation[0]
    pos_pose.pose.orientation.y = current_rotation[1]
    pos_pose.pose.orientation.z = current_rotation[2]
    pos_pose.pose.orientation.w = current_rotation[3]
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
    # marker.pose.orientation.x = target_rotation[0]
    # marker.pose.orientation.y = target_rotation[1]
    # marker.pose.orientation.z = target_rotation[2]
    # marker.pose.orientation.w = target_rotation[3]

    MARKER_PUB.publish(marker)


    time = vrpn.header.stamp.to_sec()

    message = Twist()

    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200


    # Calc values



# x, y, z, rotation
    p = [calc_p_control(kpx, target[0], position[0]),
            calc_p_control(kpy, target[1], position[1]),
            calc_p_control(kpz, target[2], position[2]),
            calc_p_control_angle(kp_rotation, target_rotation, current_rotation)]

    i = [0, 0, 0, 0]

    d = [calc_d_control(kdx, target[0], position[0], prev_position[0], time,
            prev_time, 0),
            calc_d_control(kdy, target[1], position[1], prev_position[1], time,
            prev_time, 1),
            calc_d_control(kdz, target[2], position[2], prev_position[2], time,
            prev_time, 2), 0]

    offset_angle = calc_offset_angle(current_rotation)

    rot_p = global_to_drone_coordinates(p, offset_angle)
    rot_i = global_to_drone_coordinates(i, offset_angle)
    rot_d = global_to_drone_coordinates(d, offset_angle)

    message.linear.x = rot_p[0] + rot_i[0] + rot_d[0]
    message.linear.y = rot_p[1] + rot_i[1] + rot_d[1]
    message.linear.z = rot_p[2] + rot_i[2] + rot_d[2]
    message.angular.z = rot_p[3] + rot_i[3] + rot_d[3]

    # print "%f\t%f\t%f\t%f\t%f" % (p[0], p[1], d[0], d[1], p[3])

    # print message.angular.z

    now = Time.now()
    # print now.to_sec() - startTime
    TWIST_PUB.publish(message)
    now = Time.now()
    # print now.to_sec() - startTime

    path.poses.append(pos_pose)
    PATH_PUB.publish(path)

    global prev_position
    prev_position = position

    global prev_time
    prev_time = time


def wand_handler(vrpn):

    wand_position = [vrpn.pose.position.z, vrpn.pose.position.x,
            vrpn.pose.position.y]

    wand_current_rotation = [vrpn.pose.orientation.z,
                -vrpn.pose.orientation.x, vrpn.pose.orientation.y,
                vrpn.pose.orientation.w]

    if False:
        global target
        target[0] = wand_position[0]
        target[1] = wand_position[1]

        global target_rotation
        target_rotation = wand_current_rotation



if __name__ == '__main__':
    main()



    spin()
