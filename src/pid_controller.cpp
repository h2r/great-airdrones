#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf/LinearMath/Transform.h"
#include <cmath>

/**
 * PID Controller for the ARDrone
 */

class PID_Controller {
 public:
  PID_Controller() {
    twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pos_pub =
      n.advertise<geometry_msgs::PoseStamped>("ardrone/true_position", 1);
    vrpn_sub = n.subscribe("vrpn_client_node/ardrone/pose", 1,
                           &PID_Controller::vrpn_callback, this);
    curr_pose_set_sub = n.subscribe("ardrone/current_position", 1,
                                    &PID_Controller::curr_pose_set_callback, this);

    ROS_INFO_STREAM("Made subscriber");

    kp[0] = 0.31;
    kp[1] = 0.31;
    kp[2] = 1;
    kp[3] = 0.5;

    kd[0] = -0.27;
    kd[1] = -0.27;
    kd[2] = 1;
    kd[3] = 0;

    for (int i = 0; i < 3; i++) {
      first_d[i] = true;
      a1[i] = 0.0;
      a0[i] = 0.0;
      t1[i] = 0.0;
      t0[i] = 0.0;
      true_pos[i] = 0.0;
    }

    current_pos[0] = 1;
    current_pos[1] = -1;
    current_pos[2] = 1;
    current_rot = tf::Quaternion(1, 0, 0, 0);
    true_rot = tf::Quaternion(0, 0, 0, 1);
  }
  /**
   * Converts from global to drone coordinates
   */
  void global_to_drone_coordinates(float mat[4], float angle,
                                   float new_mat[4]) {
    new_mat[0] = cos(angle) * mat[0] + sin(angle) * mat[1];
    new_mat[1] = cos(angle) * mat[1] - sin(angle) * mat[0];
    new_mat[2] = mat[2];
    new_mat[3] = mat[3];
  }

  /**
   * Calculates p
   */
  float calc_p_control(float kp, float current_val, float true_val) {
    return kp * (current_val - true_val);
  }

  /**
   * Calculates d
   */
  float calc_d_control(float kd, float current, double time, int axis) {
    float coef1 = 0.9;
    float coef0 = 0.925;

    /* ROS_INFO_STREAM(first_d[axis]); */
    if (first_d[axis]) {
      a1[axis] = current;
      a0[axis] = current;
      t1[axis] = time;
      t0[axis] = time;

      first_d[axis] = false;
    } else {
      a1[axis] = coef1 * a1[axis] + (1.0 - coef1) * current;
      a0[axis] = coef0 * a0[axis] + (1.0 - coef0) * current;
      t1[axis] = coef1 * t1[axis] + (1.0 - coef1) * time;
      t0[axis] = coef0 * t0[axis] + (1.0 - coef0) * time;
    }

    float numerator = a1[axis] - a0[axis];
    float denominator = t1[axis] - t0[axis];

    float val = kd * numerator / denominator;
    if (std::isnan(val)) {
      return 0.0;
    } else {
      return val;
    }
  }

  /**
   * Calculates the offset angle
   */
  float calc_offset_angle(tf::Quaternion current) {
    tf::Quaternion x_axis = tf::Quaternion(1, 0, 0, 0);
    tf::Quaternion a = x_axis * current.inverse();
    tf::Quaternion rotation = (a * x_axis) * a.inverse();
    float angle = atan2(rotation[1], rotation[0]);
    return angle;
  }

  /**
   * Calculates p for rotation
   */
  float calc_p_control_angle(float kp, float current_rot,
                             tf::Quaternion true_rot) {
    float angle = calc_offset_angle(true_rot);
    float after_p = calc_p_control(kp, current_rot, angle);
    return after_p;
  }

  /**
   * The vrpn_callback that handles driving the drone to the current_pos
   * position
   */
  void vrpn_callback(const geometry_msgs::PoseStamped vrpn) {
    /* Getting true_pos and rot*/
    true_pos[0] = vrpn.pose.position.z;
    true_pos[1] = vrpn.pose.position.x;
    true_pos[2] = vrpn.pose.position.y;

    true_rot[0] = vrpn.pose.orientation.z;
    true_rot[1] = -vrpn.pose.orientation.x;
    true_rot[2] = vrpn.pose.orientation.y;
    true_rot[3] = vrpn.pose.orientation.w;

    /* Publishing true_pos */
    geometry_msgs::PoseStamped pos_pose;
    pos_pose.pose.position.x = true_pos[0];
    pos_pose.pose.position.y = true_pos[1];
    pos_pose.pose.position.z = true_pos[2];
    pos_pose.pose.orientation.x = true_rot[0];
    pos_pose.pose.orientation.y = true_rot[1];
    pos_pose.pose.orientation.z = true_rot[2];
    pos_pose.pose.orientation.w = true_rot[3];
    pos_pose.header.stamp = vrpn.header.stamp;
    pos_pose.header.frame_id = "world";
    pos_pub.publish(pos_pose);

    /* Generating twist */
    geometry_msgs::Twist twist_msg;

    /* Arbitrary values to disable hover mode */
    twist_msg.angular.x = 50;
    twist_msg.angular.y = 200;

    double roll, pitch, yaw;
    tf::Matrix3x3(current_rot).getRPY(roll, pitch, yaw);

    float p[4] = {calc_p_control(kp[0], current_pos[0], true_pos[0]),
                  calc_p_control(kp[1], current_pos[1], true_pos[1]),
                  calc_p_control(kp[2], current_pos[2], true_pos[2]),
                  calc_p_control_angle(kp[3], pitch, true_rot)};

    double time = vrpn.header.stamp.toSec();

    float d[4] = {calc_d_control(kd[0], true_pos[0], time, 0),
                  calc_d_control(kd[1], true_pos[1], time, 1), 0, 0};

    float offset_angle = calc_offset_angle(true_rot);

    /* ROS_INFO_STREAM(p[0] << ", " << p[1] << ", " << p[2] << ", " << p[3]); */
    /* ROS_INFO_STREAM(true_pos[0] << ", " << true_pos[1] << ", " << true_pos[2]); */
    ROS_INFO_STREAM(current_pos[0] << ", " << current_pos[1] << ", " << current_pos[2]);

    float rot_p[4], rot_d[4];
    global_to_drone_coordinates(p, offset_angle, rot_p);
    global_to_drone_coordinates(d, offset_angle, rot_d);

    twist_msg.linear.x = rot_p[0] + rot_d[0];
    twist_msg.linear.y = rot_p[1] + rot_d[1];
    twist_msg.linear.z = rot_p[2] + rot_d[2];
    twist_msg.angular.z = rot_p[3] + rot_d[3];

    twist_pub.publish(twist_msg);

  }

  void curr_pose_set_callback(const geometry_msgs::Pose input_current_pos) {
    current_pos[0] = input_current_pos.position.x;
    current_pos[1] = input_current_pos.position.y;
    current_pos[2] = input_current_pos.position.z;
    if (1.0 == input_current_pos.orientation.x +
        input_current_pos.orientation.y + input_current_pos.orientation.z +
        input_current_pos.orientation.w) {
    current_rot[0] = input_current_pos.orientation.x;
    current_rot[1] = input_current_pos.orientation.y;
    current_rot[2] = input_current_pos.orientation.z;
    current_rot[3] = input_current_pos.orientation.w;
    }
  }

 private:
  ros::NodeHandle n;
  ros::Subscriber vrpn_sub;
  ros::Subscriber curr_pose_set_sub;
  ros::Publisher twist_pub;
  ros::Publisher pos_pub;
  float kp[4];
  float kd[4];

  bool first_d[3];
  float a1[3];
  float a0[3];
  double t1[3];
  double t0[3];

  float current_pos[3];
  tf::Quaternion current_rot;
  float true_pos[3];
  tf::Quaternion true_rot;
};

/**
 * main function. starts everything
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_controller");

  ROS_INFO_STREAM("About to make object");
  PID_Controller pid_controller;
  ROS_INFO_STREAM("Made object");

  ros::spin();

  return 0;
}
