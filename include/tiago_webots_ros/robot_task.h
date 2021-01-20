#pragma once

// tiago_webots_ros
#include "types.h"

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <gmapping/slam_gmapping.h>
#include <nav_msgs/Odometry.h>

// std
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>     
#include <math.h>   

// webots_ros
#include <webots_ros/set_int.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/lidar_get_layer_point_cloud.h>
#include <webots_ros/RecognitionObject.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>

namespace tiago_webots_ros {

class RobotTask {
  // robot
  ros::NodeHandle nh_;
  std::string robot_model_;
  LidarInfo lidar_info_;
  geometry_msgs::PointStamped position_;
  webots_ros::RecognitionObject objects_;
  sensor_msgs::Imu imu_;
  geometry_msgs::Twist vel_;
  float wheel_distance_;
  float wheel_radius_;
  float max_vel_;
  int step_;
  nav_msgs::Odometry robot_pose_odom;

  // subscribers and services
  ros::ServiceClient lidar_srv_;
  ros::Subscriber recognition_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber gyro_sub_;
  ros::Subscriber wheel_left_sub_;
  ros::Subscriber wheel_right_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub;

  // slam
  std::unique_ptr<SlamGMapping> gm;

  // odom
  std::thread odom_thread;
  std::atomic<bool> odom_enabled;
  webots_ros::Float64Stamped right_wheel_;
  webots_ros::Float64Stamped left_wheel_;
  

  void getRobotModel(const std_msgs::String::ConstPtr& name);
  void updateRightJoint(const webots_ros::Float64Stamped& joint);
  void updateLeftJoint(const webots_ros::Float64Stamped& joint);
  void updateOdom();
  void updateVel(const geometry_msgs::Twist& vel);
  void updatePosition(const geometry_msgs::PointStamped& position);
  void updateGyro(const sensor_msgs::Imu& imu);
  void updateObjects(const webots_ros::RecognitionObject& objects);
  void enableDevices(bool enable = true);

  void setTF() const;
  void initSlamGmapping();
  
  public: 
    RobotTask(ros::NodeHandle& nh);
    ~RobotTask();

    void enableLidar(bool enable);
    void enableCamera(bool enable);
    void enableGPS(bool enable);
    void enableGyro(bool enable);
    void enableWheel(bool enable);
    void getMaxVelocity();
};

}