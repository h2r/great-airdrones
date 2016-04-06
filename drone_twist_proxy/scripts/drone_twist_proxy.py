#!/usr/bin/env python3

from rospy import Publisher
from rospy import Subscriber
from rospy import init_node
from geometry_msgs.msg import Twist


init_node('drone_twist_proxy')

pub = Publisher('/cmd_vel', Twist, queue_size=5)

twist = None
def readPublisher(data):
    global twist
    if twist is None:
        twist = Twist()
    twist = data


Subscriber("/great_ardrones/ardrone/twist", Twist, readPublisher)

while True:
    if twist is not None:
        pub.publish(twist)
