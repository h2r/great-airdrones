#!/usr/bin/env python

import sys
import select
import termios
import tty
import time

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


velocityTStamp = ()
goal = None

gamma = 0.01
bound = 0.001

def callback(data):
    cur = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    if goal is not None:
	velocities = [gamma*(i - j) for i,j in zip(goal, cur)]
	velocities] = [min(val, bound)  for val in velocities]
	velocities] = [max(val, -bound) for val in velocities]
	velocityTStamp = (velocities + time.clock())


settings = termios.tcgetattr(sys.stdin)

pub = rospy.Publisher('cmd_vel', Twist)
land_pub = rospy.Publisher('/ardrone/land', Empty)
reset_pub = rospy.Publisher('/ardrone/reset', Empty)
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

rospy.init_node('great-ardrones')
print rospy.Subscriber("/vrpn_client_node/Drone/pose", PoseStamped, callback)

try:
    while(True):
        print velocityTStamp
        key = getKey()

        # takeoff and landing
        print key
        if key == "q":
            quit()
        elif key == 'l':
            land_pub.publish(Empty())
        elif key == 't':
            takeoff_pub.publish(Empty())

        twist = Twist()
        if key == "g":
            key = getKey()
            cur = ""
            temp = []

            while key != "e" and len(temp) < 3:
                if key == "-" or key.isdigit() or key == ".":
                    cur += key
                elif key == " ":
                    try:
                        temp.append(float(cur))
                    except Exception as e:
                        print e

                    cur = ""
                key = getKey()

            if len(temp) == 3:
                goal = temp
            else:
                print("New goal incorrectly formatted %.6f" % temp)

            print("Goal is %.6f" % goal)

        elif len(velocityTStamp) == 2 and time.clock() - velocityTStamp[3] < 1:
            twist.linear.x = velocityTStamp[0]
            twist.linear.y = velocityTStamp[1]
            twist.linear.z = velocityTStamp[2]

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
