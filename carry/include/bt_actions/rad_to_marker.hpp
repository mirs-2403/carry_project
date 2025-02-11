#ifndef RAD_TO_MARKER_HPP
#define RAD_TO_MARKER_HPP

#include "rclcpp/rclcpp.hpp"
#include <ros2_aruco_interfaces/srv/get_marker_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class RadToMarker : public SyncActionNode
{
public:
    // コンストラクタ
    RadToMarker(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {

    }

    // ノードの初期化や終了処理（必要に応じて）
    static PortsList providedPorts(){
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<std::int32_t>("camera_id"),
            OutputPort<std::double_t>("rad")
        };
    };

    // tick()メソッド（Behavior Treeでの実行ロジック）
    NodeStatus tick() override 
    {
        // パラメータの取得
        auto node = getInput<rclcpp::Node::SharedPtr>("node");
        auto camera_id = getInput<std::int32_t>("camera_id");
        node_ = node.value();
        

        client_ = node_->create_client<ros2_aruco_interfaces::srv::GetMarkerPose>("get_marker_pose");

        while(!client_->wait_for_service(std::chrono::seconds(1))){
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
        /*
        if (!client_->wait_for_service(std::chrono::seconds(60))) {
            RCLCPP_ERROR(node_->get_logger(), "Service not available");
            return BT::NodeStatus::FAILURE;
        }
        */
        auto request = std::make_shared<ros2_aruco_interfaces::srv::GetMarkerPose::Request>();
        request->camera_id = 1;

        auto result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto res = result.get();
            if (res->success) {
                RCLCPP_INFO(node_->get_logger(), "Service call successful");
                // 受信したposeを表示
                RCLCPP_INFO(node_->get_logger(), "Marker ID: %d", res->marker_id);
                auto marker_pose = res->pose;

                RCLCPP_INFO(node_->get_logger(), "Pose position - x: %f, y: %f, z: %f",
                            res->pose.pose.position.x,
                            res->pose.pose.position.y,
                            res->pose.pose.position.z);
                RCLCPP_INFO(node_->get_logger(), "Pose orientation - x: %f, y: %f, z: %f, w: %f",
                            res->pose.pose.orientation.x,
                            res->pose.pose.orientation.y,
                            res->pose.pose.orientation.z,
                            res->pose.pose.orientation.w);

                // marker_poseのz座標を0に設定
                marker_pose.pose.position.z = 0.0;
                marker_pose.pose.orientation.z = 0.0;

                // マーカーのx座標とy座標からラジアンを計算
                double rad = atan2(marker_pose.pose.position.y, marker_pose.pose.position.x);

                rad -= 3.1415926/2;

                // 出力ポートに設定
                RCLCPP_INFO(node_->get_logger(), "rad : %f", rad);
                setOutput("rad", rad);

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
    rclcpp::Client<ros2_aruco_interfaces::srv::GetMarkerPose>::SharedPtr client_; // サービスクライアント
    geometry_msgs::msg::PoseStamped received_pose;
};

#endif