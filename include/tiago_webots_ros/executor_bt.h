#pragma once

// tiago_webots_ros
#include <tiago_webots_ros/LoadTree.h>
#include <tiago_webots_ros/BtState.h>
#include <tiago_webots_ros/actions/move_base_planner.h>

// std
#include <chrono>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

// behaviortree_cpp_v3
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>

namespace tiago_webots_ros {
namespace tiago_behaviour_tree {

class ExecutorBT {
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  
  ros::ServiceServer srv_load_path_, srv_unload_;
  ros::Publisher status_pub_;
  
  std::string behaviour_name_;
    
  std::thread thread_;
  std::atomic<bool> exit_;
  std::mutex mutex_;
  
  BT::NodeStatus status_;
  uint32_t tree_id_;

public:
  ExecutorBT(const std::shared_ptr<ros::NodeHandle>& nh);
  ~ExecutorBT();
  BT::NodeStatus GetStatus();
  bool Load(const std::string& tree_xml);
  bool Unload();

private:
  void execute(const std::string& tree_xml);
  bool Load(LoadTreeRequest& req, LoadTreeResponse& res);
  bool Unload(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
};

} // namespace tiago_behaviour_tree
} // namespace tiago_webots_ros