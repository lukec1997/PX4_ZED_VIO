/**
 * This tutorial demonstrates simple receipt of ZED odom messages over the ROS system.
 */

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <ros/ros.h>
#include <zed_interfaces/reset_odometry.h>
#include <zed_interfaces/set_pose.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>

#define RAD2DEG 57.295779513

ros::Publisher mavros_odom_pub;
ros::Publisher status_pub;

ros::Publisher zed_odom_pub;

geometry_msgs::Point prev_position;
ros::Time prev_time;

ros::ServiceClient set_position;

double prev_roll = 0;
double prev_pitch = 0;
double prev_yaw = 0;

double x = 0;
double y = 0;
double z = 0;

double prev_x = 0;
double prev_y = 0;
double prev_z = 0;

mavros_msgs::CompanionProcessStatus status_msg;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    	
  // Get current position, current time, and calulate time difference
  geometry_msgs::Point current_position = msg->pose.pose.position;
  
  ros::Time current_time = msg->header.stamp;
  ros::Duration dt = current_time - prev_time;

  // Camera position in map frame ## Change these back to x and comment out other part if wrong!
  double cx = current_position.y * -1;
  double cy = current_position.x;
  double cz = current_position.z;

  // Velocity calculations
  double vx = (current_position.x - prev_position.x) / dt.toSec();
  double vy = (current_position.y - prev_position.y) / dt.toSec();
  double vz = (current_position.z - prev_position.z) / dt.toSec();
  
  const int cov_matrix_size = 36;
  std::array<double, cov_matrix_size> covariance_data;
  for (int i = 0; i < cov_matrix_size; i++) {
     covariance_data[i] = msg->pose.covariance[i];
  }
  
  double dx = cx - prev_x;
  double dy = cy - prev_y;
  double dz = cz - prev_z;
  
  zed_interfaces::reset_odometry resetOdom;
  zed_interfaces::set_pose setPose;
  setPose.request.x = prev_x;
  setPose.request.y = prev_y;
  setPose.request.z = prev_z;
  
  prev_position = current_position;
  double px = prev_position.y * -1;
  double py = prev_position.x;
  double pz = prev_position.z;
  
  prev_x = px;
  prev_y = py;
  prev_z = pz;
  
  prev_time = current_time;

  // Orientation quaternion
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);

  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);

  // Roll Pitch and Yaw from rotation matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double roll_speed = (roll - prev_roll) / dt.toSec();
  double pitch_speed =((pitch - prev_pitch) / dt.toSec()) * -1;
  double yaw_speed = (yaw - prev_yaw) / dt.toSec();

  prev_roll = roll;
  prev_pitch = pitch;
  prev_yaw = yaw;
  
  // Output the measure
  ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
           msg->header.frame_id.c_str(), cx, cy, cz, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

  ROS_INFO("Velocity: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz);

  //Create an Odometry message for MAVROS
  nav_msgs::Odometry mavros_odom; 

  mavros_odom.header = msg->header;

  mavros_odom.child_frame_id = "base_link";

  mavros_odom.pose.pose.position.x = cx;
  mavros_odom.pose.pose.position.y = cy;
  mavros_odom.pose.pose.position.z = cz;

  mavros_odom.twist.twist.linear.x = vx;
  mavros_odom.twist.twist.linear.y = vy;
  mavros_odom.twist.twist.linear.z = vz;

  mavros_odom.twist.twist.angular.x = roll_speed;
  mavros_odom.twist.twist.angular.y = pitch_speed;
  mavros_odom.twist.twist.angular.z = yaw_speed;

  mavros_odom.pose.covariance[0] = covariance_data[0];
  mavros_odom.pose.covariance[7] = covariance_data[7];
  mavros_odom.pose.covariance[14] = covariance_data[14];
  mavros_odom.pose.covariance[21] = covariance_data[21];
  mavros_odom.pose.covariance[28] = covariance_data[28];
  mavros_odom.pose.covariance[35] = covariance_data[35];

  //Publish the mavros_odom message
  mavros_odom_pub.publish(mavros_odom);

}

void statusCallback(const ros::TimerEvent& event)
{
  // Create and publish the MAVROS CompanionProcessStatus message
//  mavros_msgs::CompanionProcessStatus status_msg::
  status_msg.header.stamp = ros::Time::now();
 // status_msg.header.seq++;
  status_msg.component = 197; // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
 // status_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
  status_pub.publish(status_msg);
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "zed_to_mavros_publisher");

  ros::NodeHandle n;

  ros::Subscriber subOdom = n.subscribe("/zed/zed_node/odom", 10, odomCallback);

  mavros_odom_pub = n.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
  status_pub = n.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);
  
  ros::ServiceClient reset_odom = n.serviceClient<zed_interfaces::reset_odometry>("/zed/zed_node/reset_odometry");

  ros::Timer status_timer = n.createTimer(ros::Duration(1.0), statusCallback);

  prev_position.x = 0;
  prev_position.y = 0;
  prev_position.z = 0;
  prev_time = ros::Time::now();
  
  ros::spin();

  return 0;
}
