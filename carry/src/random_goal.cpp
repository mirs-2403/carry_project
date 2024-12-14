#include "bt_actions/random_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>

RandomGoalGenerateAction::RandomGoalGenerateAction(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus RandomGoalGenerateAction::tick()
{
  // ランダムな座標を生成
  geometry_msgs::msg::PoseStamped random_goal = generateRandomGoal();

  // Blackboardにgoalをセット
  setOutput("goal",random_goal);

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList RandomGoalGenerateAction::providedPorts()
{
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};  // 必要に応じてポートを定義
}

geometry_msgs::msg::PoseStamped RandomGoalGenerateAction::generateRandomGoal()
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";  // 適切なフレームを設定
    pose.header.stamp = rclcpp::Clock().now(); // 現在時刻

    // ランダムにx, y座標を生成
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-2.0, 2.0);  // -10mから+10mの範囲で

    pose.pose.position.x = dis(gen);
    pose.pose.position.y = dis(gen);
    pose.pose.position.z = 0.0;  // Zは通常0（2Dナビゲーションの場合）

    // 姿勢を設定 (クォータニオン)
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;  // 方向は無視

    return pose;
}

void RandomGoalGenerateAction::setGoalInBlackboard(const geometry_msgs::msg::PoseStamped & goal)
{
  // Blackboardにgoalを格納
    setOutput("goal", goal);
}
