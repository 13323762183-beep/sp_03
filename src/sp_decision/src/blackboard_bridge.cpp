#include "sp_decision/blackboard_bridge.hpp"

namespace sp_decision
{

BlackboardBridge::BlackboardBridge(
    BT::Blackboard::Ptr blackboard,
    const rclcpp::NodeOptions &options)
    : Node("blackboard_bridge", options),
      blackboard_(blackboard)
{
  using namespace std::placeholders;

  sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(1).reliable(),
      std::bind(&BlackboardBridge::on_goal_pose, this, _1));

  init_all_variables();
  RCLCPP_INFO(this->get_logger(), "[BlackboardBridge] Initialized (goal_pose only).");
}

void BlackboardBridge::set_blackboard(BT::Blackboard::Ptr blackboard)
{
  std::lock_guard<std::mutex> lk(bb_mutex_);
  blackboard_ = blackboard;
  if (blackboard_) {

    blackboard_->set<double>("clicked_point_update", 0.0);
    blackboard_->set<double>("nav_active", 0.0);
    blackboard_->set<int>("nav_success", 1);
    blackboard_->set<geometry_msgs::msg::PoseStamped>(
        "clicked_point", geometry_msgs::msg::PoseStamped());
  }
}

void BlackboardBridge::init_all_variables()
{
  std::lock_guard<std::mutex> lk(bb_mutex_);
  if (!blackboard_) {
    return;
  }
  blackboard_->set<double>("clicked_point_update", 0.0);
  blackboard_->set<double>("nav_active", 0.0);
  blackboard_->set<int>("nav_success", 1);
  blackboard_->set<geometry_msgs::msg::PoseStamped>(
      "clicked_point", geometry_msgs::msg::PoseStamped());
}

void BlackboardBridge::on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  write<geometry_msgs::msg::PoseStamped>("clicked_point", *msg);
  write<double>("clicked_point_update", 1.0);
  RCLCPP_INFO(this->get_logger(),
      "[BlackboardBridge] New goal_pose  x=%.3f  y=%.3f",
      msg->pose.position.x, msg->pose.position.y);
}

}
