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
  CheckCallService(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
  
private:
  rclcpp::Service<mirs_msgs::srv::NumberCommand>::SharedPtr service_;
  std::shared_ptr<rclcpp::Node> node_;
  void setBlackboard(const std::shared_ptr<mirs_msgs::srv::NumberCommand::Request> request, std::shared_ptr<mirs_msgs::srv::NumberCommand::Response> response);
};

#endif