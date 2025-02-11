#ifndef CHECK_MARKER_HPP
#define CHECK_MARKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

using namespace BT;

class CheckMarker : public SyncActionNode
{
public:
    CheckMarker(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {

    }

    static PortsList providedPorts(){
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            OutputPort<std::double_t>("x")
        };
    };

    NodeStatus tick() override
    {   
        // パラメータの取得
        auto node = getInput<rclcpp::Node::SharedPtr>("node");
        node_ = node.value();

        // サブスクライブ用のコールバック変数
        std::shared_ptr<geometry_msgs::msg::PoseStamped> received_pose;
        bool message_received = false;

        // サブスクライバの作成
        auto subscriber = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "aruco_pose_in_base_link", 10,
            [&received_pose, &message_received](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                received_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
                message_received = true;
            });

        RCLCPP_INFO(node_->get_logger(), "Node is created");

        // 一定時間待機してメッセージを受信
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(60); // タイムアウト設定

        while (rclcpp::ok() && !message_received) {
            rclcpp::spin_some(node_);
            if ((std::chrono::steady_clock::now() - start_time) > timeout) {
                RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for message");
                return NodeStatus::FAILURE;
            }
        }

        // 受信したメッセージのx座標を取得し、出力ポートに設定
        setOutput("x", received_pose->pose.position.x);

        RCLCPP_INFO(node_->get_logger(), "Received message");
        
        // 受信したメッセージのx座標を取得し、値が一定以下ならSUCCESSを返す
        if (abs(received_pose->pose.position.x) < 0.01) {
            RCLCPP_INFO(node_->get_logger(), "x is less than 0.5");
            return NodeStatus::SUCCESS;
        }else{
            RCLCPP_INFO(node_->get_logger(), "x is more than 0.5");
            return NodeStatus::FAILURE;
        }
        
        return NodeStatus::FAILURE;
    };



private:
    std::shared_ptr<rclcpp::Node> node_;
};

#endif  // CHECK_MARKER_HPP
