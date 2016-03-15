#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String

import sys, select, termios, tty, time, numpy as np


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


global velocityTStamp
velocityTStamp = ()
goal = np.array([0, 0, 0])
global oldLoc
oldLoc = np.array([1, 1, 1])

def callback(data):
    rospy.loginfo("Current Location %s",data.data)
    cur = np.array(data.data)
    if np.sum(np.abs(cur-oldLoc))==0:
    	land_pub.publish(Empty())
    	print "out of motion capture rig"
    else:
	   global oldLoc
	   oldLoc = cur
	   gamma = 0.1
	   velocities = (goal-cur)*gamma
	   bound = 0.5
	   velocities[velocities>bound] = bound
	   velocities[velocities<-bound] = -bound
	   global velocityTStamp
	   velocityTStamp = (velocities, time.clock())



print "1"
settings = termios.tcgetattr(sys.stdin)

pub = rospy.Publisher('cmd_vel', Twist)
land_pub = rospy.Publisher('/ardrone/land', Empty)
reset_pub = rospy.Publisher('/ardrone/reset', Empty)
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

rospy.init_node('drone_teleop')
#rospy.Subscriber("/Drone/pose", String, callback)
print "2"

try:
    while(True):
        key = getKey()
        # takeoff and landing
        print key
        if key=="q":
            quit()
        if key == 'l':
            land_pub.publish(Empty())
        elif key == 't':
            takeoff_pub.publish(Empty())

        twist = Twist()
        if key=="g":
            key = getKey()
            cur = ""
            temp = []
            while key!="e":
                if key!="-" and not key.isdigit():
                    cur += key
                else:
                    temp.append(int(cur))
                    cur = ""
                key = getKey()
        	goal = np.array(temp)
        elif len(velocityTStamp)==2 and time.clock()-velocityTStamp[1]<1:
        	LINEAR.x, LINEAR.y, LINEAR.z = velocityTStamp[0]
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
