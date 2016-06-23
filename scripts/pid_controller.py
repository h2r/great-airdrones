"""A module that is a pid controller for the ardrone inside the vrpn grid"""

from math import sin, cos, atan2
from rospy import init_node, Subscriber, spin, Publisher, Time
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_inverse, quaternion_multiply, \
    euler_from_quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Empty

init_node('pid_controller')

TWIST_PUB = Publisher('/cmd_vel', Twist, queue_size=1)
MARKER_PUB = Publisher('/ardrone/target_position', PoseStamped, queue_size=1)
POS_PUB = Publisher('/ardrone/current_position', PoseStamped, queue_size=1)
PATH_PUB = Publisher('/ardrone/previous_path', Path, queue_size=1)
TAKEOFF_PUB = Publisher('/ardrone/takeoff', Empty, queue_size=1)
LAND_PUB = Publisher('/ardrone/land', Empty, queue_size=1)

# Setting up default variables
# x, y, z, rotation
target = [0, 0, 1, 0]

position = [0, 0, 0, 0]

path = Path()
path.header.frame_id = "world"

# x, y, z, rotation
kp = [0.31, 0.31, 1, 0.5]
kd = [-0.27, -0.27, 0, 0]

# Variables for d calculations

first_d = [True, True, True]

a1 = [None, None, None]
a0 = [None, None, None]
t1 = [None, None, None]
t0 = [None, None, None]

def global_to_drone_coordinates(mat, angle):
    """Converts from global to drone coordinates"""
    return [cos(angle) * mat[0] + sin(angle) * mat[1],
            cos(angle) * mat[1] - sin(angle) * mat[0], mat[2], mat[3]]


def calc_offset_angle(current):
    """Calculates the offset angle from the x axis positiion"""

    x_axis = [1, 0, 0, 0]
    a = quaternion_multiply(x_axis, quaternion_inverse(current))
    rotation = quaternion_multiply(quaternion_multiply(a, x_axis), quaternion_inverse(a))
    angle = atan2(rotation[1], rotation[0])

    return angle


def calc_p_control_angle(beta, target, current):
    """Calculates p for the angle"""

    angle = calc_offset_angle(current)
    after_p = calc_p_control(beta, target, angle)
    return after_p


def calc_p_control(kp, target, current):
    """Calculates p"""
    return kp * (target - current)


def calc_d_control(kd, target, current, time, axis):
    """Calculates d"""
    coef1 = 0.9
    coef0 = 0.925

    if first_d[axis]:
        global a1
        a1[axis] = current
        global a0
        a0[axis] = current
        global t0
        t0[axis] = time
        global t1
        t1[axis] = time

        global first_d
        first_d[axis] = False

    else:
        global a1
        a1[axis] = coef1 * a1[axis] + (1.0 - coef1) * current

        global a0
        a0[axis] = coef0 * a0[axis] + (1.0 - coef0) * current

        global t1
        t1[axis] = coef1 * t1[axis] + (1.0 - coef1) * time

        global t0
        t0[axis] = coef0 * t0[axis] + (1.0 - coef0) * time


    numerator = a1[axis] - a0[axis]
    denominator = t1[axis] - t0[axis]
    return kd * numerator / denominator


def vrpn_handler(vrpn):
    """Handles the vrpn input"""

    global position
    position = [vrpn.pose.position.z, vrpn.pose.position.x,
            vrpn.pose.position.y]

    current_rotation = [vrpn.pose.orientation.z,
                -vrpn.pose.orientation.x, vrpn.pose.orientation.y,
                vrpn.pose.orientation.w]

    pos_pose = PoseStamped()
    pos_pose.pose.position.x = position[0]
    pos_pose.pose.position.y = position[1]
    pos_pose.pose.position.z = position[2]
    pos_pose.pose.orientation.x = current_rotation[0]
    pos_pose.pose.orientation.y = current_rotation[1]
    pos_pose.pose.orientation.z = current_rotation[2]
    pos_pose.pose.orientation.w = current_rotation[3]
    pos_pose.header.stamp = vrpn.header.stamp
    pos_pose.header.frame_id = "world"

    POS_PUB.publish(pos_pose)

    # Updating Marker Position
    marker = PoseStamped()
    marker.header.frame_id = "world"
    marker.pose.position.x = target[0]
    marker.pose.position.y = target[1]
    marker.pose.position.z = target[2]
    # marker.pose.orientation = euler_from_quaternion([0, 0, target[2]])
    MARKER_PUB.publish(marker)

    time = vrpn.header.stamp.to_sec()

    message = Twist()
    # These are set to arbitrary values to disable hover mode
    message.angular.x = 50
    message.angular.y = 200

    # x, y, z, rotation
    p = [calc_p_control(kp[0], target[0], position[0]),
         calc_p_control(kp[1], target[1], position[1]),
         calc_p_control(kp[2], target[2], position[2]),
         calc_p_control_angle(kp[3], target[3], current_rotation)]

    d = [calc_d_control(kd[0], target[0], position[0], time, 0),
         calc_d_control(kd[1], target[1], position[1], time, 1),
         calc_d_control(kd[2], target[2], position[2], time, 2), 0]

    offset_angle = calc_offset_angle(current_rotation)

    rot_p = global_to_drone_coordinates(p, offset_angle)
    rot_d = global_to_drone_coordinates(d, offset_angle)

    message.linear.x = rot_p[0] + rot_d[0]
    message.linear.y = rot_p[1] + rot_d[1]
    message.linear.z = rot_p[2] + rot_d[2]
    message.angular.z = rot_p[3] + rot_d[3]

    TWIST_PUB.publish(message)

    path.poses.append(pos_pose)
    PATH_PUB.publish(path)

def takeoff_handler(arg):
    """Handles takeoff"""

    TAKEOFF_PUB.publish(Empty())
    global target
    target = position
    target[2] = 1

def land_handler(arg):
    global target
    target = position
    LAND_PUB.publish(Empty())





def main():
    # Vrpn position subscriber
    Subscriber("/vrpn_client_node/ardrone/pose", PoseStamped,
                              vrpn_handler, queue_size=1)
    # Takeoff subscriber
    Subscriber("/ardrone/vrpn_takeoff", Empty, takeoff_handler, queue_size=1)
    # Landing subscriber
    Subscriber("/ardrone/vrpn_land", Empty, land_handler, queue_size=1)

if __name__ == '__main__':
    main()
    spin()
