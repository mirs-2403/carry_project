#ifndef CHECK_CALL_SERVICE_HPP
#define CHECK_CALL_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <mirs_msgs/srv/number_command.hpp>
#include <memory>

class CheckCallService : public BT::SyncActionNode
{
public:
  CheckCallService(const std::string & name, const BT::NodeConfiguration & conf)
    : BT::SyncActionNode(name, conf)
  {
      auto node = getInput<rclcpp::Node::SharedPtr>("node");
      if(!node){
          //nodeが見つからなかったときに例外を投げる
          throw BT::RuntimeError("Missing required input [node]");
      }

      node_ = node.value();

      service_ = node_->create_service<mirs_msgs::srv::NumberCommand>("test_service", std::bind(&CheckCallService::setBlackboard, this, std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(node_->get_logger(), "Make service");
  }

  BT::NodeStatus tick() override
  {
      return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
      return {
          BT::InputPort<rclcpp::Node::SharedPtr>("node"),
          BT::OutputPort<std::double_t>("member"),
          BT::OutputPort<std::string>("status")
      };
  }
  
private:
  rclcpp::Service<mirs_msgs::srv::NumberCommand>::SharedPtr service_;
  std::shared_ptr<rclcpp::Node> node_;

  void setBlackboard(const std::shared_ptr<mirs_msgs::srv::NumberCommand::Request> request, std::shared_ptr<mirs_msgs::srv::NumberCommand::Response> response)
  {
      RCLCPP_INFO(node_->get_logger(), "Received");
      if(setOutput("status", "1")){
          RCLCPP_INFO(node_->get_logger(), "Set Parameter");
      };
      setOutput("member", request->parameter);
      response->success = true; 
  }
};

#endif
