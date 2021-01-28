#pragma once

// tiago_webots_ros
#include "types.h"

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
#include <mutex>
#include <algorithm>

// webots_ros
#include <webots_ros/set_int.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/lidar_get_layer_point_cloud.h>
#include <webots_ros/RecognitionObject.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Int32Stamped.h>

namespace tiago_webots_ros {

class RobotTask {
  // robot
  ros::NodeHandle nh_;
  std::string robot_model_;
  LidarInfo lidar_info_;
  geometry_msgs::PointStamped gps_position_;
  nav_msgs::Odometry robot_pose_odom_;
  webots_ros::RecognitionObject objects_;
  sensor_msgs::Imu imu_;
  geometry_msgs::Twist vel_;
  float wheel_distance_;
  float wheel_radius_;
  float max_vel_;
  int step_;

  // subscribers and services
  ros::ServiceClient lidar_srv_;
  ros::Subscriber lidar_sub_;
  ros::Publisher laser_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Subscriber recognition_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber gyro_sub_;
  ros::Subscriber wheel_left_sub_;
  ros::Subscriber wheel_right_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber keyboard_sub_;
  ros::Publisher odom_pub;

  // slam
  std::unique_ptr<SlamGMapping> gm;

  // odom
  std::thread odom_thread;
  std::atomic<bool> odom_enabled;
  std::mutex odom_mutex_;
  webots_ros::Float64Stamped right_wheel_;
  webots_ros::Float64Stamped left_wheel_;
  
  /** Get the robot name. It is needed to enable and get all services/topics 
   * info. This is a subscription to the topic /model_name.
   * @param name the topic output (name of the robot)
   */
  void getRobotModel(const std_msgs::String::ConstPtr& name);
  
  /** It updates the scan info. A subscription to the Laser sensor topic.
   * @param scan the scan coming from the range sensor
   */
  void updateLaserScan(const sensor_msgs::LaserScan& scan);
  
  /** Update the right wheel encoder. A subscription to the right wheel sensor
   * topic.
   * @param joint the encoder info
   */
  void updateRightWheelEncoder(const webots_ros::Float64Stamped& joint);
  
  /** Update the left wheel encoder. A subscription to the right wheel sensor
   * topic.
   * @param joint the encoder info
   */
  void updateLeftWheelEncoder(const webots_ros::Float64Stamped& joint);
  
  /** Update the odometry, publishing the result to the /odom topic. Simple math
   * to convert the encoders' info to robot displacement.
   */
  void updateOdom();
  
  /** Update the robot's velocity. A subscription to the cmd_vel topic, updating
   * the velocity command to the robot's wheels.
   * @param vel the velocity to be changed
   */
  void updateVel(const geometry_msgs::Twist& vel);
  
  /** Update the robot's global position. A subscription to the gps topic. This 
   * should not be used for indoor environment.
   * @param position that comes from the gps sensor
   */
  void updateGPSPosition(const geometry_msgs::PointStamped& position);
  
  /** Update the robot's orientation. A subscription to the gyro topic.
   * @param imu info that comes from the gyro sensor
   */
  void updateOrientation(const sensor_msgs::Imu& imu);
  
  /** Update the objects on robot's sight. A subscription to recognized objects.
   * Webots has some special objects that are automatically recognized.
   * @param objects info that comes recognition topic
   */
  void updateRecognizedObjects(const webots_ros::RecognitionObject& objects);

  /** Update the robot's vel given the pushed keyboard key.
   * w, d, s, a and space to stop the robot.
   * @param data keyboard's pushed key
   */
  void updateKeyboard(const webots_ros::Int32Stamped& data);
  
  /** A method to enable all useful devices for autonomous navigation.
   * 
   */ 
  void enableDevices(bool enable = true);

  
  /** Start TF base_link
   */ 
  void setTF() const;
  
  public: 
    RobotTask(ros::NodeHandle& nh);
    ~RobotTask();

    void enableLidar(bool enable);
    void enableCamera(bool enable);
    void enableGPS(bool enable);
    void enableGyro(bool enable);
    void enableWheel(bool enable);
    void enableKeyboard(bool enable);
    
    /** It returns the maximum velocity */ 
    float getMaxVelocity();

    /** Set the robot's global position 
     * @param position the global position
     */ 
    void setPosition(const geometry_msgs::PointStamped& position);
    
    /** Set the robot's global position using the GPS sensor if available.
     * It does not uptade the robot's position otherwise.
     */ 
    void setPosition();
};

}