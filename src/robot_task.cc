#include <tiago_webots_ros/robot_task.h>

namespace tiago_webots_ros {

RobotTask::RobotTask(ros::NodeHandle& nh) :
    nh_(nh) {
  auto sub = nh_.subscribe("/model_name", 100, &RobotTask::getRobotModel, 
    this);
  
  while (robot_model_.empty()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);
  enableDevices(true);
  setTF();
  initSlamGmapping();
}

RobotTask::~RobotTask() {
  lidar_srv_.shutdown();
  recognition_sub_.shutdown();
  gps_sub_.shutdown();
  gyro_sub_.shutdown();
  wheel_left_sub_.shutdown();
  wheel_right_sub_.shutdown();
  odom_enabled = false;
  if (odom_thread.joinable())
    odom_thread.join();
}


void RobotTask::getRobotModel(const std_msgs::String::ConstPtr& name) {
  robot_model_ = name->data;
}

void RobotTask::updateRightJoint(const webots_ros::Float64Stamped& joint) {
  right_wheel_ = joint;
}

void RobotTask::updateLeftJoint(const webots_ros::Float64Stamped& joint) {
  left_wheel_ = joint;
}

void RobotTask::updatePosition(const geometry_msgs::PointStamped& position) {
  position_ = position;
}

void RobotTask::updateObjects(const webots_ros::RecognitionObject& objects) {
  objects_ = objects;
}

void RobotTask::updateGyro(const sensor_msgs::Imu& imu) {
  imu_ = imu;
}

void RobotTask::updateOdom() {
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  tf::TransformBroadcaster odom_broadcaster;
  float wheel_radius = 0.1;
  float dist_between_wheel = 0.4044;
  int dt = 35;
  float r = right_wheel_.data;
  float l = left_wheel_.data;
  float yaw = 0;
  
  while (odom_enabled) {
    std::this_thread::sleep_for(std::chrono::milliseconds(dt));
    // calculating new position given wheels' encoders
    float dr = (r - right_wheel_.data) * wheel_radius;
    float dl = (l - left_wheel_.data) * wheel_radius;
    float dyaw = (dr - dl) / dist_between_wheel;
    float ds = (dr + dl) / 2;
    float dx = ds * cos(yaw + dyaw / 2);
    float dy = ds * sin(yaw + dyaw / 2);;
    
    // update position
    position.x += dx;
    position.y += dy;
    yaw += dyaw;
    orientation = tf::createQuaternionMsgFromYaw(yaw);
    auto current_time = ros::Time::now();

    // publishing tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = position.x;
    odom_trans.transform.translation.y = position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = orientation;
    odom_broadcaster.sendTransform(odom_trans);

    //publishing the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = position.x;
    odom.pose.pose.position.y = position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = dy / dt;
    odom.twist.twist.angular.z = dyaw / dt;
    odom_pub.publish(odom);
  }
}


void RobotTask::enableDevices(bool enable) {
  enableLidar(enable);
  enableWheel(enable);
  enableCamera(enable);
  enableGPS(enable);
  enableGyro(enable);
}

void RobotTask::setTF() const {
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = robot_model_ + "/Hokuyo_URG_04LX_UG01";
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);
}

void RobotTask::initSlamGmapping() {
  ros::NodeHandle slam_nh;
  slam_nh.setParam("scan_topic", robot_model_ + 
    "/Hokuyo_URG_04LX_UG01/laser_scan/layer0");
  gm = std::make_unique<SlamGMapping>(nh_, slam_nh);
  gm->startLiveSlam();
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
  if (enable) {
    wheel_left_sub_ = nh_.subscribe(robot_model_ + 
      "/wheel_left_joint_sensor/value", 100, 
      &RobotTask::updateLeftJoint, this);
    wheel_right_sub_ = nh_.subscribe(robot_model_ + 
      "/wheel_right_joint_sensor/value", 100, 
      &RobotTask::updateRightJoint, this);
    odom_enabled = true;
    odom_thread = std::thread(&RobotTask::updateOdom, this);
  }
  else {
    wheel_left_sub_.shutdown();
    wheel_right_sub_.shutdown();
    odom_enabled = false;
    if (odom_thread.joinable())
      odom_thread.join();
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
