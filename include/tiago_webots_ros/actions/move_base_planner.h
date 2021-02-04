#pragma once

// tiago_webots_ros
#include <tiago_webots_ros/actions/planner_node.h>
#include <tiago_webots_ros/ros_bt_ports.h>

// ros
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace tiago_webots_ros {
namespace tiago_behaviour_tree {

class MoveBasePlanner : public PlannerNode<move_base_msgs::MoveBaseAction, 
  geometry_msgs::PoseStamped> {
protected:
  nav_msgs::Path executed_path_;

public:
  MoveBasePlanner(const std::string &name, const BT::NodeConfiguration &config, 
    const std::shared_ptr<ros::NodeHandle>& nh);
  ~MoveBasePlanner() = default;
  static BT::PortsList providedPorts();

protected:
  void on_start() override;
  virtual void on_tick() override;
  void init(const geometry_msgs::PoseStamped& goal);
  void on_new_goal_received() override;

private:
  void doneCb(const actionlib::SimpleClientGoalState& state);
  void activeCb();
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  
};

} // namespace tiago_behaviour_tree
} // namespace tiago_webots_ros