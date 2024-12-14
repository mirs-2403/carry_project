#ifndef IS_CURRENT_POSE_HPP
#define IS_CURRENT_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class IsCurrentPose : public SyncActionNode
{
public:
    // コンストラクタ
    IsCurrentPose(const std::string& name, const NodeConfiguration& config);

    // tick()メソッド（Behavior Treeでの実行ロジック）
    NodeStatus tick() override;

    // ノードの初期化や終了処理（必要に応じて）
    static PortsList providedPorts();

private:
    void markerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    rclcpp::Node::SharedPtr node_; // ROS 2ノード
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_; // サブスクライバ
    bool is_current_pose = false;
};

#endif