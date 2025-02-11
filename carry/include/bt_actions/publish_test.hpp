#ifndef PUBLISH_TEST_HPP
#define PUBLISH_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace BT;

class PublishTest : public SyncActionNode
{
public:
    PublishTest(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {

    }

    static PortsList providedPorts(){
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<geometry_msgs::msg::PoseStamped>("goal")
        };
    };

    NodeStatus tick() override
    {   

        // パラメータの取得
        auto node = getInput<rclcpp::Node::SharedPtr>("node");
        auto pose = getInput<geometry_msgs::msg::PoseStamped>("goal");
        node_ = node.value();

        // パブリッシャーの作成
        auto publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);

        max_publish_count_ = 10;
        publish_count_ = 0;

        while (publish_count_ < max_publish_count_ && rclcpp::ok()) {
            publisher->publish(pose.value());
            publish_count_++;
            RCLCPP_INFO(node_->get_logger(), "Published %d/%d", publish_count_, max_publish_count_);

            // 適切な間隔を持たせるためにスリープ
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (publish_count_ >= max_publish_count_) {
            publisher.reset();
            RCLCPP_INFO(node_->get_logger(), "Footprint Publisher Stopped after %d publishes", max_publish_count_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    };



private:
    rclcpp::Node::SharedPtr node_; // ROS 2ノード
    geometry_msgs::msg::PoseStamped received_pose;
    int publish_count_;
    int max_publish_count_;
};

#endif  // PUBLISH_TEST_HPP
