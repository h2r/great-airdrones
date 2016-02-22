#!/usr/bin/env python
import roslib; roslib.load_manifest('test_flight')
import rospy
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

if __name__=="__main__":
	
	pub = rospy.Publisher('cmd_vel', Twist)
	land_pub = rospy.Publisher('/ardrone/land', Empty)
	reset_pub = rospy.Publisher('/ardrone/reset', Empty)
	takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

	rospy.init_node('test_flight')

	try:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                print "sleepingn"
                time.sleep(5)
                print "takeoff"
                takeoff_pub.publish(Empty())
                for x in range(0, 100):
                    time.sleep(0.1)
                    pub.publish(twist)
                land_pub.publish(Empty())
                for x in range(0, 100):
                    time.sleep(0.1)
                    pub.publish(twist)
	except Exception as e:
		print e
		print repr(e)
