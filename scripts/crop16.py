#!/usr/bin/python2
# -*- coding: <utf-8> -*-

import numpy as np
import cv2

import sys
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


if __name__ == '__main__':
    rospy.init_node('crop16')

    bridge = CvBridge()
    publisher = rospy.Publisher(sys.argv[2], Image, queue_size=5)

    def getimg(data):
        cv2img = bridge.imgmsg_to_cv2(data, "bgr8")
        shape = list(cv2img.shape)

        shape[0] = shape[0]//16*16
        shape[1] = shape[1]//16*16

        cv2img = cv2img[:shape[0], :shape[1]]
        publisher.publish(bridge.cv2_to_imgmsg(cv2img, "bgr8"))

    subscriber = rospy.Subscriber(sys.argv[1], Image, getimg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down...')
