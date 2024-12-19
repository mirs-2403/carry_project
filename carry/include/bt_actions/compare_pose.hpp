#ifndef COMPARE_POSE_HPP
#define COMPARE_POSE_HPP

#include <memory>
#include <string>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace BT
{

class ComparePose : public SyncActionNode {
public:
    ComparePose(const std::string& name, const NodeConfiguration& config, rclcpp::Node::SharedPtr ros_node)
        : SyncActionNode(name, config), node_(ros_node)
    {
        if (!node_)
        {
        throw std::runtime_error("Shared node is null.");
        }
    }

    static BT::PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped>("target_pose") 
        };
    }

    BT::NodeStatus tick() override 
    {
        // 入力ポートから対象のposeを取得
        geometry_msgs::msg::PoseStamped target_pose;
        if (!getInput("target_pose", target_pose)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get target_pose input");
            return BT::NodeStatus::FAILURE;
        }

        // サブスクライブ用のコールバック変数
        std::shared_ptr<nav_msgs::msg::Odometry> current_odom_;
        bool odom_received = false;

        // サブスクライバの作成
        auto subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odometry_subscriber", 10,
            [&current_odom_, &odom_received](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                current_odom_ = std::make_shared<nav_msgs::msg::Odometry>(*msg);
                odom_received = true;
            }
        );

        // 一定時間待機してメッセージを受信
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(5); // タイムアウト設定

        // クォータニオンをベクトルに変換
        tf2::Quaternion robot_quat(
            current_odom_->pose.pose.orientation.x,
            current_odom_->pose.pose.orientation.y,
            current_odom_->pose.pose.orientation.z,
            current_odom_->pose.pose.orientation.w);

        tf2::Quaternion target_quat(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w);

        tf2::Vector3 robot_dir = quaternionToVector(robot_quat);
        tf2::Vector3 target_dir = quaternionToVector(target_quat);

        // 同軸性と直交性の判定
        if (areVectorsCollinear(robot_dir, target_dir, TOLERANCE)) {
            RCLCPP_INFO(node_->get_logger(), "Robot and target are collinear (same axis)");
            return BT::NodeStatus::SUCCESS;
        } else if (areVectorsOrthogonal(robot_dir, target_dir, TOLERANCE)) {
            RCLCPP_INFO(node_->get_logger(), "Robot and target are orthogonal");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_WARN(node_->get_logger(), "Neither collinear nor orthogonal");
        return BT::NodeStatus::FAILURE;
    }

private:
    static constexpr double TOLERANCE = 1e-6; // 判定の許容誤差

    // クォータニオンを単位ベクトルに変換
    tf2::Vector3 quaternionToVector(const tf2::Quaternion& q) {
        tf2::Vector3 vec(1.0, 0.0, 0.0); // x軸方向の基準ベクトル
        return tf2::quatRotate(q, vec);  // クォータニオンで回転を適用
    }

    // ベクトルの内積計算
    double dotProduct(const tf2::Vector3& v1, const tf2::Vector3& v2) {
        return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
    }

    // ベクトルの大きさ（ノルム）を計算
    double magnitude(const tf2::Vector3& v) {
        return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
    }

    // ベクトルの直行性を判定
    bool areVectorsOrthogonal(const tf2::Vector3& v1, const tf2::Vector3& v2, double tolerance) {
        return std::fabs(dotProduct(v1, v2)) < tolerance;
    }

    // ベクトルの同軸性を判定
    bool areVectorsCollinear(const tf2::Vector3& v1, const tf2::Vector3& v2, double tolerance) {
        double cos_theta = dotProduct(v1, v2) / (magnitude(v1) * magnitude(v2));
        return std::fabs(std::fabs(cos_theta) - 1.0) < tolerance;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
};

}///end namespace

#endif
