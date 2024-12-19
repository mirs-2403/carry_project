#ifndef SUBSCRIBE_POSE_HPP
#define SUBSCRIBE_POSE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

namespace BT
{

class SubscribePose : public SyncActionNode
{
public:
  SubscribePose(const std::string& name, const NodeConfiguration& config, rclcpp::Node::SharedPtr ros_node)
    : SyncActionNode(name, config), node_(ros_node)
  {
    if (!node_)
    {
      throw std::runtime_error("Shared node is null.");
    }
  }

  // 提供されるポートを定義
  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>("topic_name", "Name of the topic to subscribe to"),
      OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  // 実行ロジック
  virtual NodeStatus tick() override
  {
    std::string topic_name;
    std::string output_key;

    // 入力ポートから"topic_name"を取得
    if (!getInput("topic_name", topic_name))
    {
      throw RuntimeError("Missing input port [topic_name]");
    }

    // サブスクライブ用のコールバック変数
    std::shared_ptr<geometry_msgs::msg::PoseStamped> received_pose;
    bool message_received = false;

    // サブスクライバの作成
    auto subscriber = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_name, 10,
      [&received_pose, &message_received](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        received_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
        message_received = true;
      });

    // 一定時間待機してメッセージを受信
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5); // タイムアウト設定

    while (rclcpp::ok() && !message_received)
    {
      rclcpp::spin_some(node_);
      if (std::chrono::steady_clock::now() - start_time > timeout)
      {
        RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for a message on topic: %s", topic_name.c_str());
        return NodeStatus::FAILURE;
      }
    }

    // 受信したメッセージをBlackboardに保存
    if (message_received && received_pose)
    {
      setOutput("goal", received_pose);
      RCLCPP_INFO(node_->get_logger(), "Message received and stored in Blackboard under key: %s", output_key.c_str());
      return NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to receive a message on topic: %s", topic_name.c_str());
    return NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_; // 共有ノード
};

} // namespace BT

#endif
