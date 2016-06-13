#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

import math
import sys
import signal
import numpy as np

from copy import deepcopy
from collections import deque

import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Stamp(object):
    position, time = None, None


if __name__ == '__main__':
    rospy.init_node('calc_error')

    points1, points2 = deque(), deque()
    first_val1, first_val2 = None, None

    def getdata(data, array, vrpn):
        global first_val1
        global first_val2

        elem = Stamp()
        elem.time = 1e10*data.header.stamp.secs + data.header.stamp.nsecs

        position = data.pose.position
        if vrpn:
            elem.position = np.array([position.z, -position.y, position.x])
            if first_val1 is None:
                first_val1 = deepcopy(elem.position)

            elem.position -= first_val1
        else:
            elem.position = np.array([position.x, position.y, position.z])
            if first_val2 is None:
                first_val2 = deepcopy(elem.position)

            elem.position -= first_val2

        array.append(deepcopy(elem))

    def getdata1(data):
        global points1
        getdata(data, points1, True)

    def getdata2(data):
        global points2
        getdata(data, points2, False)

    rospy.Subscriber(sys.argv[1], PoseStamped, getdata1)
    rospy.Subscriber(sys.argv[2], PoseStamped, getdata2)

    publisher = rospy.Publisher(sys.argv[3], String, queue_size=5)

    try:
        def do_exit(signum, stack):
            raise KeyboardInterrupt()

        signal.signal(signal.SIGINT, do_exit)

        rate = rospy.Rate(100)
        while True:
            rbound1 = len(points1)
            rbound2 = len(points2)

            if rbound1 == 0:
                continue
            if rbound2 <= 1:
                continue

            left, right = -1, rbound2
            while right - left > 2:
                medium = left + (right - left)//2

                current_time  = abs(points2[medium + 0].time - points1[0].time)
                previous_time = abs(points2[medium - 1].time - points1[0].time)

                if current_time < previous_time + 1e-4:
                    left = medium
                else:
                    right = medium

            # diff = points1[0].position - points2[left].position
            # publisher.publish("%.7f" % np.linalg.norm(diff))
            publisher.publish(repr(points1[0].position) + ' ' + repr(points2[left].position))

            points1.popleft()
            for _ in range(left - 1):
                points2.popleft()

            rate.sleep()

    except KeyboardInterrupt:
        print('Shutting Down...')

