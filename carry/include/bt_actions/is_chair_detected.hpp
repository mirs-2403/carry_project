#ifndef IS_CHAIR_DETECTED_HPP
#define IS_CHAIR_DETECTED_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

using namespace BT;

class IsChairDetected : public SyncActionNode
{
public:
    IsChairDetected(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {

    }

    static PortsList providedPorts(){
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            OutputPort<geometry_msgs::msg::PoseStamped>("goal")
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
            "chair_pose", 10,
            [&received_pose, &message_received](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                received_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
                message_received = true;
            });

        RCLCPP_INFO(node_->get_logger(), "Node is created");

        // 一定時間待機してメッセージを受信
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(20); // タイムアウト設定

        while (rclcpp::ok() && !message_received) {
            rclcpp::spin_some(node_);
            if ((std::chrono::steady_clock::now() - start_time) > timeout) {
                RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for message");
                return NodeStatus::FAILURE;
            }
        }

        // 受信したメッセージのx座標を取得し、出力ポートに設定
        setOutput("goal", *received_pose);

        RCLCPP_INFO(node_->get_logger(), "Received message");
        
        return NodeStatus::SUCCESS;
    };



private:
    std::shared_ptr<rclcpp::Node> node_;
};

#endif  // IS_CHAIR_DETECTED_HPP
