#ifndef PUBLISH_CHAIR_POSITION_HPP
#define PUBLISH_CHAIR_POSITION_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace BT;

class PublishChairPosition : public SyncActionNode
{
public:
    PublishChairPosition(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {

    }

    static PortsList providedPorts(){
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<geometry_msgs::msg::PoseStamped>("goal1"),
            InputPort<geometry_msgs::msg::PoseStamped>("goal2"),
            InputPort<geometry_msgs::msg::PoseStamped>("goal3"),
            InputPort<geometry_msgs::msg::PoseStamped>("goal4"),
            InputPort<geometry_msgs::msg::PoseStamped>("goal5"),
        };
    };

    NodeStatus tick() override
    {   

        // パラメータの取得
        auto node = getInput<rclcpp::Node::SharedPtr>("node");
        auto pose1 = getInput<geometry_msgs::msg::PoseStamped>("goal1");
        auto pose2 = getInput<geometry_msgs::msg::PoseStamped>("goal2");
        auto pose3 = getInput<geometry_msgs::msg::PoseStamped>("goal3");
        auto pose4 = getInput<geometry_msgs::msg::PoseStamped>("goal4");
        auto pose5 = getInput<geometry_msgs::msg::PoseStamped>("goal5");
        node_ = node.value();

        // visualizetion_msgs::msg::MarkerArray型にarrowとして変換
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose1.value().pose;
        marker.scale.x = 0.1;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker marker2;
        marker2.header.frame_id = "map";
        marker2.header.stamp = rclcpp::Clock().now();
        marker2.ns = "basic_shapes";
        marker2.id = 1;
        marker2.type = visualization_msgs::msg::Marker::ARROW;
        marker2.action = visualization_msgs::msg::Marker::ADD;
        marker2.pose = pose2.value().pose;
        marker2.scale.x = 0.1;
        marker2.scale.y = 0.01;
        marker2.scale.z = 0.01;
        marker2.color.a = 1.0;
        marker2.color.r = 1.0;
        marker2.color.g = 0.0;
        marker2.color.b = 0.0;
        marker_array.markers.push_back(marker2);

        visualization_msgs::msg::Marker marker3;
        marker3.header.frame_id = "map";
        marker3.header.stamp = rclcpp::Clock().now();
        marker3.ns = "basic_shapes";
        marker3.id = 2;
        marker3.type = visualization_msgs::msg::Marker::ARROW;
        marker3.action = visualization_msgs::msg::Marker::ADD;
        marker3.pose = pose3.value().pose;
        marker3.scale.x = 0.1;
        marker3.scale.y = 0.01;
        marker3.scale.z = 0.01;
        marker3.color.a = 1.0;
        marker3.color.r = 1.0;
        marker3.color.g = 0.0;
        marker3.color.b = 0.0;
        marker_array.markers.push_back(marker3);

        visualization_msgs::msg::Marker marker4;
        marker4.header.frame_id = "map";
        marker4.header.stamp = rclcpp::Clock().now();
        marker4.ns = "basic_shapes";
        marker4.id = 3;
        marker4.type = visualization_msgs::msg::Marker::ARROW;
        marker4.action = visualization_msgs::msg::Marker::ADD;
        marker4.pose = pose4.value().pose;
        marker4.scale.x = 0.1;
        marker4.scale.y = 0.01;
        marker4.scale.z = 0.01;
        marker4.color.a = 1.0;
        marker4.color.r = 1.0;
        marker4.color.g = 0.0;
        marker4.color.b = 0.0;
        marker_array.markers.push_back(marker4);

        visualization_msgs::msg::Marker marker5;
        marker5.header.frame_id = "map";

        marker5.header.stamp = rclcpp::Clock().now();
        marker5.ns = "basic_shapes";
        marker5.id = 4;
        marker5.type = visualization_msgs::msg::Marker::ARROW;
        marker5.action = visualization_msgs::msg::Marker::ADD;
        marker5.pose = pose5.value().pose;
        marker5.scale.x = 0.1;
        marker5.scale.y = 0.01;
        marker5.scale.z = 0.01;
        marker5.color.a = 1.0;
        marker5.color.r = 1.0;
        marker5.color.g = 0.0;
        marker5.color.b = 0.0;
        marker_array.markers.push_back(marker5);





        // パブリッシャーの作成
        auto publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_test", 10);

        max_publish_count_ = 10;
        publish_count_ = 0;

        while (publish_count_ < max_publish_count_ && rclcpp::ok()) {
            publisher->publish(marker_array);
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

#endif  // PUBLISH_CHAIR_POSITION_HPP
