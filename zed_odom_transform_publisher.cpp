#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
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

    // Subscribers
    _vslam_odom_sub = nh.subscribe("/zed/zed_node/odom", 10, &VioTransform::odometryCallback, this);
    _fc_imu_sub = nh.subscribe("/zed/zed_node/imu/data", 10, &VioTransform::sensorCombinedCallback, this);
  }

private:
  // Subscription callbacks
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void sensorCombinedCallback(const sensor_msgs::Imu::ConstPtr& msg);

  // Publishers
  ros::Publisher _vio_pub;
  ros::Publisher _imu_pub;

  // Subscribers
  ros::Subscriber _vslam_odom_sub;
  ros::Subscriber _fc_imu_sub;
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
  tf2::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  tf2::Vector3 angular_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

  // Assuming ZED publishes in FLU world frame AKA NWU (north west up)
  tf2::Quaternion rotation;
  rotation.setRPY(M_PI, 0.0, 0.0);

  position = tf2::quatRotate(rotation, position);
  quaternion = rotation * quaternion * rotation.inverse();
  velocity = tf2::quatRotate(rotation, velocity);
  angular_velocity = tf2::quatRotate(rotation, angular_velocity);

  // Fill the message
  nav_msgs::Odometry vio;
  vio.header.stamp = ros::Time::now();
  vio.header.frame_id = "odom";
  vio.child_frame_id = "base_link";

  vio.pose.pose.position.x = position.getX();
  vio.pose.pose.position.y = position.getY();
  vio.pose.pose.position.z = position.getZ();

  vio.pose.pose.orientation.x = quaternion.getX();
  vio.pose.pose.orientation.y = quaternion.getY();
  vio.pose.pose.orientation.z = quaternion.getZ();
  vio.pose.pose.orientation.w = quaternion.getW();

  vio.twist.twist.linear.x = velocity.getX();
  vio.twist.twist.linear.y = velocity.getY();
  vio.twist.twist.linear.z = velocity.getZ();

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vio_transform");

  VioTransform vio_transform;

  ros::spin();

  return 0;
}
