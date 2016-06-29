from math import sqrt
from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Pose, PoseStamped

init_node('screensaver')
target_pub = Publisher('/ardrone/curr_pose', Pose, queue_size=1)
true_pos = [None, None, None]
center = [1, -1, 1]
target = [0, 0, 1]

centered = False

def calc_distance(pos1, pos2):
    val = 0
    for i in range(0, 3):
        val += (pos1[i] - pos2[i]) ** 2
    return sqrt(val)

def true_pos_handler(msg):
    true_pos[0] = msg.pose.position.x
    true_pos[1] = msg.pose.position.y
    true_pos[2] = msg.pose.position.z

    dist_from_center = calc_distance(true_pos, center)

    print dist_from_center

    if abs(center[0] - true_pos[0]) > 0.3:
        target[0] = (center[0] - abs(true_pos[0])) * true_pos[0] / \
                abs(true_pos[0]) * -1

    if abs(center[1] - true_pos[1]) > 0.3:
        target[1] = (center[1] - abs(true_pos[1])) * true_pos[1] / \
                abs(true_pos[1]) * -1

    message = Pose()
    message.position.x = target[0]
    message.position.y = target[1]
    message.position.z = 1

    target_pub.publish(message)


def main():
    Subscriber("/ardrone/true_position", PoseStamped, true_pos_handler,
               queue_size=1)

    original = Pose()
    original.position.x = 1
    original.position.y = -1
    original.position.z = 1
    print "PUBLISHED"
    target_pub.publish(original)


if __name__ == "__main__":
    main()
    spin()
