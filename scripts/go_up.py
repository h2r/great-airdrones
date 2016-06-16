from rospy import init_node, Subscriber, spin, Publisher, Time
from geometry_msgs.msg import Twist, PoseStamped


init_node('go_up')
TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)

while True:
    message = Twist()
    message.linear.z = 5.0
    TWIST_PUB.publish(message)
