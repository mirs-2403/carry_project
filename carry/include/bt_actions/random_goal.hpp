#ifndef RANDOM_GOAL_HPP
#define RANDOM_GOAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

class RandomGoalGenerateAction : public BT::SyncActionNode
{
public:
  RandomGoalGenerateAction(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  geometry_msgs::msg::PoseStamped generateRandomGoal();
  void setGoalInBlackboard(const geometry_msgs::msg::PoseStamped & goal);
};

#endif  // RANDOM_NAVIGATE_TO_POSE_ACTION_HPP
