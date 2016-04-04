#!/usr/bin/env python

import sys
import select
import termios
import tty
import time
import re
import math

import roslib
roslib.load_manifest('great-ardrones')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


global latestGridData
latestGridData = None
global lastSeq
lastSeq = -1
global timeElapsed
timeElapsed = 0
goal = None
gamma = 0.10
bound = 0.1
initialYaw = None

def callback(data):
    global latestGridData
    latestGridData = [data.pose.position.x, data.pose.position.y,
    data.pose.position.z, data.pose.orientation.y, data.header.seq]



settings = termios.tcgetattr(sys.stdin)

pub = rospy.Publisher('cmd_vel', Twist)
land_pub = rospy.Publisher('/ardrone/land', Empty)
reset_pub = rospy.Publisher('/ardrone/reset', Empty)
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)
rospy.init_node('great-ardrones')
rospy.Subscriber("/vrpn_client_node/ardrone/pose", PoseStamped, callback)


def computeVelocities(goal, cur):
    velocities = [gamma*(i - j) for i,j in zip(goal, cur)]
    for i, v in enumerate(velocities):
        if abs(v) < 0.001:
            velocities[i] = 0
    velocities = [min(val, bound)  for val in velocities]
    velocities = [max(val, -bound) for val in velocities]
    return velocities

try:
    while(True):
        key = getKey()
        print key
        twist = Twist()
        if key == "q":
            quit()
        elif key=="r":
            reset_pub.publish(Empty())
        elif key=="c":
            goal = latestGridData[:3]
            print "goal is current location", goal
        elif key=="d":
            goal = None
        elif key == 'l':
            land_pub.publish(Empty())
        elif key == 't':
            takeoff_pub.publish(Empty())
        elif key == "g":
            inputString = ""
            while key != "e":
                key = getKey()
                inputString += key
            inputString = inputString[:-1]
            try:
                x, y, z = re.findall(r"[-+]?\d*\.\d+|\d+", inputString)
                goal = [float(x), float(y), float(z)]
                print "Successfully set new goal", goal
            except Exception as e:
                print e
                print "New goal incorrectly formatted", inputString
                print "The goal remains ", goal
        elif latestGridData is not None:
            """
            The drone must be carefully initialized in the optitrak system.
            Ensure that the following conditions hold:
                1) Moving forward increases the drone's x position in the grid
                2) Moving left decreases the drone's z position in the grid
                3) Moving up decreases the drone's y position in the grid
                4) When in a position satisfying 1-3 the drone's angles
                   are all 0

            These conditions can be satisfied by correctly aligning the
            drone with the optitrak co-ordinate system and then initializing
            it in the software so that the angles are set to 0.
            """
            if initialYaw is None:
                initialYaw = latestGridData[3]

            if abs(time.clock() - timeElapsed) >= 0.01:
                if lastSeq == latestGridData[4]:
                    print "Gone out of the grid. Move back in"
                    pub.publish(Twist())
                    land_pub.publish(Empty())
                else:
                    lastSeq = latestGridData[4]
                timeElapsed = time.clock()

            elif goal is not None:
                currentYaw = latestGridData[3]-initialYaw
                velX, velY, velZ = computeVelocities(goal, latestGridData[:3])
                twist.linear.x = velX#(velX*math.cos(currentYaw) + velZ*math.sin(currentYaw))
                twist.linear.y = -velZ#velX*math.sin(currentYaw) - velZ*math.cos(currentYaw)
                twist.linear.z = velY

        print "twist = ", twist
        #print goal
        pub.publish(twist)
except Exception as e:
    try:
	print repr(e)
	print e
	land_pub.publish(Empty())
	twist = Twist()
	pub.publish(twist)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
