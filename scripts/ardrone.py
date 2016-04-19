#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

"""ARDrone controlling class."""


from copy import deepcopy
import threading
import math
import time
import signal
import rospy

import numpy as np
import cv2

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

def clamp(value, limit):
    return max(min(value, limit), -limit)


# class TwistThread(threading.Thread):
    # """Thread constantly republishing the last received twist."""

    # def __init__(self):
    #     """Constructor."""
    #     super(TwistThread, self).__init__()
    #     self.__twist = Twist()

    #     self.daemon = True
    #     self.__stop = False

    # def stop(self):
    #     """Set stop flag to True."""
    #     self.__stop = True

    # def resettwist(self, message):
    #     """Set new twist for publishing."""
    #     self.__twist = deepcopy(message)

    # def run(self):
    #     """Republish the last received twist."""
    #     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    #     rate = rospy.Rate(10)
    #     while True:
    #         if not self.__stop:
    #             pub.publish(self.__twist)
    #         else:

    #             break

    #         rate.sleep()


class ARDroneThread(threading.Thread):
    """ARDrone thread to run time-consuming commands."""

    def __init__(self, drone, name, args):
        """Constructor."""
        super(ARDroneThread, self).__init__()

        self.daemon = True
        self.__stop = False

        self.__name = name
        self.__args = args
        self.__drone = drone

        self.__retstatus = ""

    def stop(self):
        """Set stop flag to True."""
        self.__stop = True

    def isstopped(self):
        """Return stop status."""
        return self.__stop

    def returnstatus(self):
        """Get return status from the stopped thread."""
        return self.__retstatus

    def run(self):
        """Wrap the method to call."""
        func = getattr(self.__drone, self.__name)
        self.__retstatus = func(*self.__args)


class ARDrone:
    """ARDrone controlling class."""

    def __init__(self, name):
        """Constructor."""
        def emptypublisher(name):
            return rospy.Publisher(name, Empty, queue_size=1)

        # self.__bridge = CvBridge()

        self.__reset = emptypublisher('/' + name + '/reset')
        self.__land = emptypublisher('/' + name + '/land')
        self.__takeoff = emptypublisher('/' + name + '/takeoff')
        self.__twistpublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.__ardronethread = None

        self.__position = None
        self.__rotation = None

        self.__status = 'landed'
        self.__vrpnsequence = None

        self.__time_elapsed = time.clock()
        self.__last_seq = -1
        self.__out_of_the_grid = True

        def getvrpndata(vrpndata):
            """Get vrpn position data and store into the class member."""
            point = vrpndata.pose.position
            angles = vrpndata.pose.orientation
            self.__rotation = [3.14*angles.x, -3.14*angles.z, 3.14*angles.y]

            self.__position = [-point.x, point.z, point.y]
            self.__vrpnsequence = vrpndata.header.seq

            if time.clock() - self.__time_elapsed >= 0.01:
                if self.__last_seq == self.__vrpnsequence:
                    self.__out_of_the_grid = True

                else:
                    self.__out_of_the_grid = False
                    self.__last_seq = self.__vrpnsequence

                self.__time_elapsed = time.clock()

        self.__getvrpndata = getvrpndata

        vrpntopic = '/vrpn_client_node/' + name + '/pose'
        rospy.Subscriber(vrpntopic, PoseStamped, self.__getvrpndata)

        # def getimg(data):
        #     self.__cv2img = self.__bridge.imgmsg_to_cv2(data, "bgr8")
            # gray = cv2.cvtColor(self.__cv2img, cv2.COLOR_BGR2GRAY)

            # sift = cv2.xfeatures2d.SIFT_create()
            # kp = sift.detect(gray, None)

            # cv2.drawKeypoints(gray, kp, self.__cv2img)
            # cv2.imshow("Frontal camera", self.__cv2img)

            # cv2.waitKey(3)

        # topic = '/' + name + '/front/image_raw'
        # self.__imgstream = rospy.Subscriber(topic, Image, getimg)

    def destroy(self):
        """Destructor."""
        pass

    def takeoff(self):
        """Take the drone off."""
        self.__status = 'flying'
        self.__takeoff.publish(Empty())

    def land(self):
        """Land the drone."""
        self.settwist([0, 0, 0], [0, 0, 0])
        self.__land.publish(Empty())
        self.__status = 'landed'

    def reset(self):
        """Reset the drone."""
        self.settwist([0, 0, 0], [0, 0, 0])
        self.__reset.publish(Empty())
        self.__status = "landed"

    def settwist(self, linear, angular):
        """Publish twist."""
        message = Twist()

        message.linear.x = linear[0]
        message.linear.y = linear[1]
        message.linear.z = linear[2]

        message.angular.x = angular[0]
        message.angular.y = angular[1]
        message.angular.z = angular[2]

        print(message.linear)
        self.__twistpublisher.publish(message)

    def getposition(self):
        """Return the drone global position."""
        if self.__out_of_the_grid:
            return None
        else:
            return deepcopy(self.__position)

    def move(self, diff, turnfirst):
        """Move drone to the position relative to the current one."""
        if self.__status == "landed":
            self.takeoff()
            time.sleep(5)

        cur_pos = self.getposition()

        wherepoint = [0, 0, 0]
        for i in range(3):
            wherepoint[i] = cur_pos[i] + diff[i]

        return self.goto(wherepoint, turnfirst)

    def gotoasync(self, wherepoint, turnfirst):
        """Make the drone going to <wherepoint>."""
        droneposition = deepcopy(self.__position)
        if droneposition is None:
            return "I can't get any vrpn data related to the drone"

        if self.__status == 'landed':
            self.takeoff()
            time.sleep(5)

        rate = rospy.Rate(10)

        while not self.__ardronethread.isstopped():
            if self.__out_of_the_grid:
                self.land()
                return 'I just went out of the grid and landed'

            rate.sleep()

            droneposition = deepcopy(self.__position)

            twistlinear = [0.0, 0.0, 0.0]
            twistangular = [0.0, 0.0, 0.0]

            dx = wherepoint[0] - droneposition[0]
            dy = wherepoint[1] - droneposition[1]
            dz = wherepoint[2] - droneposition[2]

            if max([abs(dx), abs(dy), abs(dz)]) < 0.1:
                self.land()
                break

            if turnfirst:
                twistangular[2] = math.atan2(dy, dx) - self.__rotation[2]
                twistangular[2] = clamp(twistangular[2], 0.4)

                if abs(twistangular[2]) < 0.04:
                    twistlinear[0] = math.sqrt(dx*dx + dy*dy)
                twistlinear[0] = clamp(twistlinear[0], 0.05)

                if max(abs(dx), abs(dy)) < 0.05:
                    twistlinear[2] = clamp(dz, 0.05)

            else:
                if max(abs(dx), abs(dy)) < 0.05:
                    twistlinear = [0, 0, dz]
                else:
                    twistlinear = [dx, dy, 0]
                    print(twistlinear)

                twistlinear = [clamp(dv, 0.05) for dv in twistlinear]


            self.settwist(twistlinear, twistangular)


        if self.__ardronethread.isstopped():
            return 'Interrupted'

        return 'success'

    def goto(self, wherepoint, turnfirst):
        """Make the drone going to <wherepoint>."""
        def do_exit(signum, stack):
            raise KeyboardInterrupt()

        signal.signal(signal.SIGINT, do_exit)

        args = (wherepoint, turnfirst)
        self.__ardronethread = ARDroneThread(self, 'gotoasync', args)
        self.__ardronethread.start()

        try:
            while self.__ardronethread.is_alive():
                self.__ardronethread.join(0.5)

        except KeyboardInterrupt:
            self.__ardronethread.stop()
            self.__ardronethread.join()
            return 'Interrupted'

        return self.__ardronethread.returnstatus()
