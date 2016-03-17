#!/usr/bin/env python
import roslib; roslib.load_manifest('great-ardrones')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

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
    print "received", data
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

rospy.init_node('great-ardrones')
#<<<<<<< HEAD
rospy.Subscriber("vrpn_client_node/Drone/pose", String, callback)
#=======
#rospy.Subscriber("vrpn_ros_node/Drone/pose", String, callback)
#>>>>>>> 6e04459de384aaba61d3c781e97f6a8752e5b2b9
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
            while key!="e" and len(temp)<3:
                if key=="-" or key.isdigit():
                    cur += key
                elif key==" ":
                    try:
                        temp.append(int(cur))
                    except Exception as e:
                        print e
                    cur = ""
                key = getKey()
            if len(temp)==3:
                goal = np.array(temp)
            else:
                print "new goal incorrectly formatted", temp
            print goal
        elif len(velocityTStamp)==2 and time.clock()-velocityTStamp[1]<1:
        	LINEAR = twist.LINEAR
                LINEAR.x, LINEAR.y, LINEAR.z = velocityTStamp[0]
        print twist
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
