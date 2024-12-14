#include "rclcpp/rclcpp.hpp"
#include "bt_actions/second_test.hpp"

using namespace BT;

SecondClientCall::SecondClientCall(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("second_test_client");
    client_ = node_->create_client<mirs_msgs::srv::SimpleCommand>("second_test_service");
}

NodeStatus SecondClientCall::tick()
{
    
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<mirs_msgs::srv::SimpleCommand::Request>();

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
            RCLCPP_INFO(node_->get_logger(), "Service call successful");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed");
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList SecondClientCall::providedPorts()
{
    return PortsList();  // 必要に応じてポートを定義
}
