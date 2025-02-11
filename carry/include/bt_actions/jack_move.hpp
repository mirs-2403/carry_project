#ifndef JACK_MOVE_HPP
#define JACK_MOVE_HPP

#include "rclcpp/rclcpp.hpp"
#include <mirs_msgs/srv/simple_command.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class JackMove : public SyncActionNode
{
public:
    // コンストラクタ
    JackMove(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {

    }

    // ノードの初期化や終了処理（必要に応じて）
    static PortsList providedPorts(){
        return {
            InputPort<std::string>("service_name"),
            InputPort<rclcpp::Node::SharedPtr>("node")
        };
    };

    // tick()メソッド（Behavior Treeでの実行ロジック）
    NodeStatus tick() override 
    {
        std::string service_name;

        auto node = getInput<rclcpp::Node::SharedPtr>("node");
        node_ = node.value();

        // 入力ポートから"service_name"を取得
        if (!getInput("service_name", service_name))
        {
            throw RuntimeError("Missing input port [topic_name]");
        }

        client_ = node_->create_client<mirs_msgs::srv::SimpleCommand>(service_name);

        while(!client_->wait_for_service(std::chrono::seconds(1))){
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
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
    };

private:
    rclcpp::Node::SharedPtr node_; // ROS 2ノード
    rclcpp::Client<mirs_msgs::srv::SimpleCommand>::SharedPtr client_; // サービスクライアント
};

#endif
