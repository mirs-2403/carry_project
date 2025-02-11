#ifndef IS_CALLED_SERVICE_HPP
#define IS_CALLED_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <carry_msgs/srv/chair_arrangement.hpp>
#include <memory>

class IsCalledService : public BT::SyncActionNode
{
public:
  IsCalledService(const std::string & name, const BT::NodeConfiguration & conf)
    : BT::SyncActionNode(name, conf)
  {
      auto node = getInput<rclcpp::Node::SharedPtr>("node");
      if(!node){
          //nodeが見つからなかったときに例外を投げる
          throw BT::RuntimeError("Missing required input [node]");
      }

      node_ = node.value();

      service_ = node_->create_service<carry_msgs::srv::ChairArrangement>("chair_arrangement_service", std::bind(&IsCalledService::setBlackboard, this, std::placeholders::_1, std::placeholders::_2));
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
          BT::OutputPort<std::int32_t>("member"),
          BT::OutputPort<std::string>("status"),
          BT::OutputPort<std::int32_t>("location")
      };
  }
  
private:
    rclcpp::Service<carry_msgs::srv::ChairArrangement>::SharedPtr service_;
    std::shared_ptr<rclcpp::Node> node_;

    void setBlackboard(const std::shared_ptr<carry_msgs::srv::ChairArrangement::Request> request, std::shared_ptr<carry_msgs::srv::ChairArrangement::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received");
        if(request->arrangement_pattern == 2){
            setOutput("status", "0");
        }else if(request->arrangement_pattern == 3){
            setOutput("status", "1");
        }else{
            RCLCPP_INFO(node_->get_logger(), "Worrong arrangement type");
            setOutput("status", "2");
            response->success = false;
        }
        setOutput("member", request->number_of_people);
        setOutput("location", request->location);
        RCLCPP_INFO(node_->get_logger(), "Set Parameter");
        response->success = true;
    }
};

#endif
