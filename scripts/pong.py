from time import sleep
from math import sqrt
from rospy import init_node, Subscriber, spin, Publisher
from geometry_msgs.msg import Pose, PoseStamped

init_node('pong')
target_pub = Publisher('/ardrone/curr_pose', Pose, queue_size=1)
true_pos = [None, None, None]
paddle_1_pos = [5, 5, 5]
paddle_2_pos = [None, None, None]
origin = [1, -1, 1]
target = [1, -1, 1]
vel = [0.005, 0.005, 0]

def calc_distance(obj1, obj2):
    return_val = 0
    for i in range(0, 3):
        return_val += (obj1[i] - obj2[i]) ** 2
    return sqrt(return_val)

def true_pos_handler(msg):
    # Update Position
    true_pos[0] = msg.pose.position.x
    true_pos[1] = msg.pose.position.y
    true_pos[2] = msg.pose.position.z

    for i in range(0, 3):
        target[i] += vel[i]
    if abs(target[1] - origin[1]) > 0.5:
        vel[1] *= -1

    print calc_distance(paddle_1_pos, true_pos)
    if calc_distance(paddle_1_pos, true_pos) < 0.1:
        vel[0] *= -1


    # Publishing Stuff
    to_pub = Pose()
    to_pub.position.x = target[0]
    to_pub.position.y = target[1]
    to_pub.position.z = target[2]

    target_pub.publish(to_pub)


def paddle_1_handler(msg):
    paddle_1_pos[0] = msg.pose.position.z
    paddle_1_pos[1] = msg.pose.position.x
    paddle_1_pos[2] = msg.pose.position.y

def paddle_2_handler(msg):
    paddle_2_pos[0] = msg.pose.position.z
    paddle_2_pos[1] = msg.pose.position.x
    paddle_2_pos[2] = msg.pose.position.y

def main():
    Subscriber("/ardrone/true_position", PoseStamped, true_pos_handler,
               queue_size=1)
    Subscriber("/vrpn_client_node/paddle_1/pose", PoseStamped, paddle_1_handler,
               queue_size=1)
    Subscriber("/vrpn_client_node/paddle_2/pose", PoseStamped, paddle_2_handler,
               queue_size=1)


if __name__ == "__main__":
    main()
    spin()
