#ifndef CANCEL_NAV2_HPP
#define CANCEL_NAV2_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <carry_msgs/srv/chair_arrangement.hpp>
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"
#include <memory>

class CancelNav2 : public BT::SyncActionNode
{
public:
  CancelNav2(const std::string & name, const BT::NodeConfiguration & conf)
    : BT::SyncActionNode(name, conf)
  {

  }

  BT::NodeStatus tick() override
  {
        auto node = getInput<rclcpp::Node::SharedPtr>("node");
        if(!node){
            //nodeが見つからなかったときに例外を投げる
            throw BT::RuntimeError("Missing required input [node]");
        }

        node_ = node.value();
        navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_,"navigate_to_pose");
        navigation_action_client_->async_cancel_all_goals();
        return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
      return {BT::InputPort<rclcpp::Node::SharedPtr>("node")};
  }
  
private:
    rclcpp::Service<carry_msgs::srv::ChairArrangement>::SharedPtr service_;
    std::shared_ptr<rclcpp::Node> node_;
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
    

};

#endif
