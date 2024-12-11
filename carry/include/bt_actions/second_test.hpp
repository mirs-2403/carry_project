#ifndef SECOND_TEST_HPP
#define SECOND_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include <mirs_msgs/srv/simple_command.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class SecondClientCall : public SyncActionNode
{
public:
    // コンストラクタ
    SecondClientCall(const std::string& name, const NodeConfiguration& config);

    // tick()メソッド（Behavior Treeでの実行ロジック）
    NodeStatus tick() override;

    // ノードの初期化や終了処理（必要に応じて）
    static PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_; // ROS 2ノード
    rclcpp::Client<mirs_msgs::srv::SimpleCommand>::SharedPtr client_; // サービスクライアント
};

#endif