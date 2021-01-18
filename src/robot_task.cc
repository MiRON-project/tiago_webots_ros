#include <tiago_webots_ros/robot_task.h>

namespace tiago_webots_ros {

RobotTask::RobotTask(ros::NodeHandle& nh) :
    nh_(nh) {
  while (robot_model_.empty()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  enableDevices(true);
}

RobotTask::~RobotTask() {
  lidar_srv_.shutdown();
  recognition_sub_.shutdown();
  gps_sub_.shutdown();
  gyro_sub_.shutdown();
  wheel_left_sub_.shutdown();
  wheel_right_sub_.shutdown();
}


void RobotTask::getRobotModel(const std_msgs::String::ConstPtr& name) {
  robot_model_ = name->data;
}

void RobotTask::updatePosition(const geometry_msgs::PointStamped& position) {
  position_.point.x = position.point.x;
  position_.point.y = position.point.y;
  position_.point.z = position.point.z;
}

void RobotTask::updateObjects(const webots_ros::RecognitionObject& objects) {
  objects_ = objects;
}

void RobotTask::updateGyro(const sensor_msgs::Imu& imu) {
  imu_ = imu;
}


void RobotTask::enableDevices(bool enable) {
  enableLidar(enable);
  enableWheel(enable);
  enableCamera(enable);
  enableGPS(enable);
  enableGyro(enable);
}

void RobotTask::enableLidar(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? 1 : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/Hokuyo_URG_04LX_UG01/enable");
  dev.call(msg);
  
  dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/Hokuyo_URG_04LX_UG01/enable_point_cloud");
  dev.call(msg);

  if (enable) {
    webots_ros::lidar_get_info info;
    info.request.ask = 1;
    dev = nh_.serviceClient<webots_ros::lidar_get_info>(robot_model_ + 
      "/Hokuyo_URG_04LX_UG01/get_info");
    dev.call(info);
    lidar_info_.fov = info.response.fov;
    lidar_info_.horizontalResolution = info.response.horizontalResolution;
    lidar_info_.maxRange = info.response.maxRange;
    lidar_info_.minRange = info.response.minRange;
    lidar_info_.numberOfLayers = info.response.numberOfLayers;
    lidar_info_.verticalFov = info.response.verticalFov;
    lidar_srv_ = nh_.serviceClient<webots_ros::lidar_get_layer_point_cloud>(
      robot_model_ + "/Hokuyo_URG_04LX_UG01/lidar_get_layer_point_cloud");
  }
}

void RobotTask::enableWheel(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? 1 : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/wheel_right_joint_sensor/enable");
  dev.call(msg);
  dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/wheel_left_joint_sensor/enable");
  dev.call(msg);
  }
}

void RobotTask::enableCamera(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? 1 : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/camera_2D/enable");
  dev.call(msg);

  dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/camera_2D/recognition_enable");
  dev.call(msg);

  if (enable) {
    recognition_sub_ = nh_.subscribe(robot_model_ + 
      "/camera_2D/recognition_objects", 100, &RobotTask::updateObjects, this);
  }
  else {
    recognition_sub_.shutdown();
  }
}

void RobotTask::enableGPS(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? 1 : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/gps/enable");
  dev.call(msg);

  if (enable) {
    gps_sub_ = nh_.subscribe(robot_model_ + "/gps/values", 100, 
      &RobotTask::updatePosition, this);
  }
  else {
    gps_sub_.shutdown();
  }
}

void RobotTask::enableGyro(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? 1 : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/gyro/enable");
  dev.call(msg);
  
  if (enable) {
    gyro_sub_ = nh_.subscribe(robot_model_ + "/gyro/values", 100, 
      &RobotTask::updateGyro, this);
  }
  else {
    gyro_sub_.shutdown();
  }
}

} // end namespace tiago_webots_ros
