#include <tiago_webots_ros/robot_task.h>

namespace tiago_webots_ros {

RobotTask::RobotTask(ros::NodeHandle& nh) :
    nh_(nh),
    wheel_distance_(0.4044),
    wheel_radius_(0.1),
    max_vel_(0),
    step_(1) {
  
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
  cmd_vel_sub_.shutdown();
}


void RobotTask::getRobotModel(const std_msgs::String::ConstPtr& name) {
  robot_model_ = name->data;
}

void RobotTask::updateRightWheelEncoder(const webots_ros::Float64Stamped& joint) 
{
  right_wheel_ = joint;
}

void RobotTask::updateLeftWheelEncoder(const webots_ros::Float64Stamped& joint) 
{
  left_wheel_ = joint;
}

void RobotTask::updateVel(const geometry_msgs::Twist& vel) {
  vel_ = vel;
  float vx = vel.linear.x;
  float vy = vel.linear.y;
  float speed = sqrt(vx * vx + vy * vy);
  float omega = vel.angular.z;
  
  float right_wheel_vel = (wheel_distance_ * omega + 2 * speed) / 2;
  float left_wheel_vel = (2 * speed - omega * wheel_distance_) / 2;

  if (abs(left_wheel_vel) > max_vel_)
    left_wheel_vel = ((left_wheel_vel > 0) - (left_wheel_vel < 0)) * 
      max_vel_;
  if (abs(right_wheel_vel) > max_vel_)
    right_wheel_vel = ((right_wheel_vel > 0) - (right_wheel_vel < 0)) * 
      max_vel_;
  
  webots_ros::set_float msg_2;
  msg_2.request.value = right_wheel_vel;
  auto dev = nh_.serviceClient<webots_ros::set_float>(robot_model_ + 
    "/wheel_right_joint/set_velocity");
  dev.call(msg_2);

  msg_2.request.value = left_wheel_vel;
  dev = nh_.serviceClient<webots_ros::set_float>(robot_model_ + 
    "/wheel_left_joint/set_velocity");
  dev.call(msg_2);
}

void RobotTask::updatePosition(const geometry_msgs::PointStamped& position) {
  position_ = position;
}

void RobotTask::updateRecognizedObjects(
    const webots_ros::RecognitionObject& objects) {
  objects_ = objects;
}

void RobotTask::updateOrientation(const sensor_msgs::Imu& imu) {
  imu_ = imu;
}

void RobotTask::updateOdom() {
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  tf::TransformBroadcaster odom_broadcaster;
  int dt = 35;
  float r = right_wheel_.data;
  float l = left_wheel_.data;
  float yaw = 0;
  
  while (odom_enabled) {
    std::this_thread::sleep_for(std::chrono::milliseconds(dt));
    // calculating new position given wheels' encoders
    float dr = (right_wheel_.data - r) * wheel_radius_;
    float dl = (left_wheel_.data - l) * wheel_radius_;
    float dyaw = (dr - dl) / wheel_distance_;
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

    r = right_wheel_.data;
    l = left_wheel_.data;
  }
}

void RobotTask::updateKeyboard(const webots_ros::Int32Stamped& data) {
  switch (data.data)
  {
    case 87:
      vel_.linear.x += .1;
      break;
    case 68:
      vel_.angular.z += -.5;
      break;
    case 83:
      vel_.linear.x += -.1;
      break;
    case 65:
      vel_.angular.z += .5;
      break;
    case 32:
      vel_.linear.x = 0;
      vel_.linear.y = 0;
      vel_.angular.z = 0;
      break;
  }
  updateVel(vel_);
}


void RobotTask::enableDevices(bool enable) {
  enableLidar(enable);
  enableWheel(enable);
  enableCamera(enable);
  enableGPS(enable);
  enableGyro(enable);
  enableKeyboard(enable);
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
  msg.request.value = enable ? step_ : 0;
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
  msg.request.value = enable ? step_ : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/wheel_right_joint_sensor/enable");
  dev.call(msg);
  dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/wheel_left_joint_sensor/enable");
  dev.call(msg);
  if (enable) {
    wheel_left_sub_ = nh_.subscribe(robot_model_ + 
      "/wheel_left_joint_sensor/value", 100, 
      &RobotTask::updateLeftWheelEncoder, this);
    wheel_right_sub_ = nh_.subscribe(robot_model_ + 
      "/wheel_right_joint_sensor/value", 100, 
      &RobotTask::updateRightWheelEncoder, this);

    
    // set velocity to zero
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    updateVel(vel);
    
    // set the motors to veloctiy control
    webots_ros::set_float wheelSrv;
    wheelSrv.request.value = INFINITY;
    auto leftWheelPositionClient =
      nh_.serviceClient<webots_ros::set_float>(robot_model_ + 
      "/wheel_left_joint/set_position");
    leftWheelPositionClient.call(wheelSrv);
    auto rightWheelPositionClient =
      nh_.serviceClient<webots_ros::set_float>(robot_model_ + 
      "/wheel_right_joint/set_position");
    rightWheelPositionClient.call(wheelSrv);

    // add subscriber to cmd_vel
    cmd_vel_sub_ = nh_.subscribe(robot_model_ + "/cmd_vel", 100, 
      &RobotTask::updateVel, this);
    
    odom_enabled = true;
    odom_thread = std::thread(&RobotTask::updateOdom, this);
    getMaxVelocity();
  }
  else {
    wheel_left_sub_.shutdown();
    wheel_right_sub_.shutdown();
    cmd_vel_sub_.shutdown();
    odom_enabled = false;
    if (odom_thread.joinable())
      odom_thread.join();
  }
}

void RobotTask::enableCamera(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? step_ : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/camera_2D/enable");
  dev.call(msg);

  dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/camera_2D/recognition_enable");
  dev.call(msg);

  if (enable) {
    recognition_sub_ = nh_.subscribe(robot_model_ + 
      "/camera_2D/recognition_objects", 100, &RobotTask::updateRecognizedObjects,
      this);
  }
  else {
    recognition_sub_.shutdown();
  }
}

void RobotTask::enableGPS(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? step_ : 0;
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
  msg.request.value = enable ? step_ : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/gyro/enable");
  dev.call(msg);
  
  if (enable) {
    gyro_sub_ = nh_.subscribe(robot_model_ + "/gyro/values", 100, 
      &RobotTask::updateOrientation, this);
  }
  else {
    gyro_sub_.shutdown();
  }
}

void RobotTask::enableKeyboard(bool enable) {
  webots_ros::set_int msg;
  msg.request.value = enable ? step_ : 0;
  auto dev = nh_.serviceClient<webots_ros::set_int>(robot_model_ + 
    "/keyboard/enable");
  dev.call(msg);
  
  if (enable) {
    keyboard_sub_ = nh_.subscribe(robot_model_ + "/keyboard/key", 100, 
      &RobotTask::updateKeyboard, this);
  }
  else {
    keyboard_sub_.shutdown();
  }
}

void RobotTask::getMaxVelocity() {
  webots_ros::get_float msg;
  msg.request.ask = false;
  auto dev = nh_.serviceClient<webots_ros::get_float>(robot_model_ + 
    "/wheel_left_joint/get_max_velocity");
  dev.call(msg);
  ros::spinOnce();
  max_vel_ = msg.response.value;
  
  dev = nh_.serviceClient<webots_ros::get_float>(robot_model_ + 
    "/wheel_right_joint/get_max_velocity");
  dev.call(msg);
  ros::spinOnce();

  max_vel_ = std::min((float)msg.response.value, max_vel_);
}

} // end namespace tiago_webots_ros
