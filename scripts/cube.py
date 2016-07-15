from time import sleep
from math import sqrt
from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Pose, PoseStamped

init_node('cube')
target_pub = Publisher('/ardrone/curr_pose', Pose, queue_size=1)
true_pos = [None, None, None]

def true_pos_handler(msg):
    true_pos[0] = msg.pose.position.x
    true_pos[1] = msg.pose.position.y
    true_pos[2] = msg.pose.position.z


def main():
    Subscriber("/ardrone/true_position", PoseStamped, true_pos_handler,
               queue_size=1)
    while True:
        target = Pose()
        target.position.x = 1.7
        target.position.y = -0.3
        target.position.z = 1
        target_pub.publish(target)
        sleep(5)

        target = Pose()
        target.position.x = 1.7
        target.position.y = -1.3
        target.position.z = 1
        target_pub.publish(target)
        sleep(5)

        target = Pose()
        target.position.x = 0.3
        target.position.y = -1.3
        target.position.z = 1
        target_pub.publish(target)
        sleep(5)

        target = Pose()
        target.position.x = 0.3
        target.position.y = -0.3
        target.position.z = 1
        target_pub.publish(target)
        sleep(5)



if __name__ == "__main__":
    main()
    spin()
