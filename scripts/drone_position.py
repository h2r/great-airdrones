#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

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
    	LAND_PUB.publish(Empty())
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



settings = termios.tcgetattr(sys.stdin)
print "here"
pub = rospy.Publisher('cmd_vel', Twist)
land_pub = rospy.Publisher('/ardrone/land', Empty)
reset_pub = rospy.Publisher('/ardrone/reset', Empty)
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

rospy.init_node('drone_teleop')
#rospy.Subscriber("/Drone/pose", String, callback)
print "all done"
try:
    while(True):
        key = getKey()
        # takeoff and landing
        print key
	if key == 'l':
            land_pub.publish(Empty())
        elif key == 't':
            takeoff_pub.publish(Empty())

        twist = Twist()
        if key[:4]=="goal":
        	goal = np.array([float(x) for x in key[4:].strip().split()])
        elif len(velocityTStamp)==2 and time.clock()-velocityTStamp[1]<1:
        	LINEAR.x, LINEAR.y, LINEAR.z = velocityTStamp[0]
        pub.publish(twist)

except Exception as e:
    try:
	print repr(e)
	print e
	print "failed"
	land_pub.publish(Empty())
	twist = Twist()
	pub.publish(twist)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
