#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>

class VioTransform
{
public:
  VioTransform()
  {
    ros::NodeHandle nh;

    // Publishers
    _vio_pub = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
    _imu_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/data", 10);
    status_pub = nh.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);

    // Subscribers
    _vslam_odom_sub = nh.subscribe("/zed/zed_node/odom", 10, &VioTransform::odometryCallback, this);
    _fc_imu_sub = nh.subscribe("/zed/zed_node/imu/data", 10, &VioTransform::sensorCombinedCallback, this);

    // Timer for status updates
    status_timer = nh.createTimer(ros::Duration(1.0), &VioTransform::statusCallback, this);

    // Initialize previous time
    _prev_time = ros::Time::now();
  }

private:
  // Subscription callbacks
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void sensorCombinedCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void statusCallback(const ros::TimerEvent& event);

  // Publishers
  ros::Publisher _vio_pub;
  ros::Publisher _imu_pub;
  ros::Publisher status_pub;

  // Subscribers
  ros::Subscriber _vslam_odom_sub;
  ros::Subscriber _fc_imu_sub;

  // Timer
  ros::Timer status_timer;

  // Previous state
  tf2::Vector3 _prev_position;
  tf2::Quaternion _prev_orientation;
  ros::Time _prev_time;
};

void VioTransform::sensorCombinedCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_msgs::Imu imu_msg = *msg;

  imu_msg.header.stamp = ros::Time::now();
  imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  imu_msg.linear_acceleration_covariance = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };
  imu_msg.angular_velocity_covariance = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };
  
  _imu_pub.publish(imu_msg);
}

void VioTransform::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf2::Vector3 current_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  tf2::Quaternion current_orientation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  ros::Time current_time = msg->header.stamp;
  double dt = (current_time - _prev_time).toSec();

  // Calculate velocities if the time difference is valid
  tf2::Vector3 linear_velocity(0.0, 0.0, 0.0);
  tf2::Vector3 angular_velocity(0.0, 0.0, 0.0);
  if (dt > 0) {
    linear_velocity = (current_position - _prev_position) / dt;

    tf2::Matrix3x3 current_mat(current_orientation);
    tf2::Matrix3x3 prev_mat(_prev_orientation);
    tf2::Matrix3x3 delta_mat = prev_mat.transpose() * current_mat;
    tf2::Quaternion delta_q;
    delta_mat.getRotation(delta_q);

    double delta_angle = delta_q.getAngle();
    tf2::Vector3 delta_axis = delta_q.getAxis();
    angular_velocity = delta_axis * delta_angle / dt;
  }

  // Update previous state
  _prev_position = current_position;
  _prev_orientation = current_orientation;
  _prev_time = current_time;

  // Assuming ZED publishes in FLU world frame AKA NWU (north west up)
  tf2::Quaternion rotation;
  rotation.setRPY(M_PI, 0.0, 0.0);

  current_position = tf2::quatRotate(rotation, current_position);
  current_orientation = rotation * current_orientation * rotation.inverse();
  linear_velocity = tf2::quatRotate(rotation, linear_velocity);
  angular_velocity = tf2::quatRotate(rotation, angular_velocity);

  // Fill the message
  nav_msgs::Odometry vio;
  vio.header.stamp = ros::Time::now();
  vio.header.frame_id = "odom";
  vio.child_frame_id = "base_link";

  vio.pose.pose.position.x = current_position.getX();
  vio.pose.pose.position.y = current_position.getY();
  vio.pose.pose.position.z = current_position.getZ();

  vio.pose.pose.orientation.x = current_orientation.getX();
  vio.pose.pose.orientation.y = current_orientation.getY();
  vio.pose.pose.orientation.z = current_orientation.getZ();
  vio.pose.pose.orientation.w = current_orientation.getW();

  vio.twist.twist.linear.x = linear_velocity.getX();
  vio.twist.twist.linear.y = linear_velocity.getY();
  vio.twist.twist.linear.z = linear_velocity.getZ();

  vio.twist.twist.angular.x = angular_velocity.getX();
  vio.twist.twist.angular.y = angular_velocity.getY();
  vio.twist.twist.angular.z = angular_velocity.getZ();

  // Set the covariance matrices (example values, you might need to adjust these)
  for (int i = 0; i < 36; ++i)
  {
    vio.pose.covariance[i] = 0.0;
    vio.twist.covariance[i] = 0.0;
  }
  vio.pose.covariance[0] = 0.01;
  vio.pose.covariance[7] = 0.01;
  vio.pose.covariance[14] = 0.01;
  vio.pose.covariance[21] = 0.01;
  vio.pose.covariance[28] = 0.01;
  vio.pose.covariance[35] = 0.01;

  vio.twist.covariance[0] = 0.01;
  vio.twist.covariance[7] = 0.01;
  vio.twist.covariance[14] = 0.01;
  vio.twist.covariance[21] = 0.01;
  vio.twist.covariance[28] = 0.01;
  vio.twist.covariance[35] = 0.01;

  _vio_pub.publish(vio);
}

void VioTransform::statusCallback(const ros::TimerEvent& event)
{
  // Create and publish the MAVROS CompanionProcessStatus message
  mavros_msgs::CompanionProcessStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 197; // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
  status_pub.publish(status_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vio_transform");

  VioTransform vio_transform;

  ros::spin();

  return 0;
}
