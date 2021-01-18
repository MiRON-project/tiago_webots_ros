#pragma once

// tiago_webots_ros
#include "types.h"

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

// std
#include <memory>
#include <string>

// webots_ros
#include <webots_ros/set_int.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/lidar_get_layer_point_cloud.h>
#include <webots_ros/RecognitionObject.h>

namespace tiago_webots_ros {

class RobotTask {
  ros::NodeHandle nh_;
  std::string robot_model_;
  LidarInfo lidar_info_;
  geometry_msgs::PointStamped position_;
  webots_ros::RecognitionObject objects_;
  sensor_msgs::Imu imu_;

  // subscribers and services
  ros::ServiceClient lidar_srv_;
  ros::Subscriber recognition_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber gyro_sub_;
  ros::Subscriber wheel_left_sub_;
  ros::Subscriber wheel_right_sub_;
  

  void getRobotModel(const std_msgs::String::ConstPtr& name);
  void updatePosition(const geometry_msgs::PointStamped& position);
  void updateGyro(const sensor_msgs::Imu& imu);
  void updateObjects(const webots_ros::RecognitionObject& objects);
  void enableDevices(bool enable = true);
  
  public: 
    RobotTask(ros::NodeHandle& nh);
    ~RobotTask();

    void enableLidar(bool enable);
    void enableCamera(bool enable);
    void enableGPS(bool enable);
    void enableGyro(bool enable);
    void enableWheel(bool enable);
};

}