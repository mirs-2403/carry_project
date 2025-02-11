#ifndef FOOTPRINT_SETTING_HPP
#define FOOTPRINT_SETTING_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/polygon.hpp>

using namespace BT;

class FootprintSetting : public SyncActionNode
{
public:
    FootprintSetting(const std::string& name, const NodeConfiguration& config, rclcpp::Node::SharedPtr ros_node)
    : SyncActionNode(name, config), node_(ros_node)
    {
        if (!node_)
        {
            throw std::runtime_error("Shared node is null.");
        }
    }

    static PortsList providedPorts(){
        return {
            InputPort<std::float_t>("local_width"),
            InputPort<std::float_t>("local_height"),
            InputPort<std::float_t>("global_radius")
        };
    };

    NodeStatus tick() override
    {   
        // パラメータの取得
        std::float_t width;
        std::float_t height;
        std::float_t radius;

        if (!getInput("local_width", width))
        {
            throw RuntimeError("Missing input port [local_width]");
        }
        if (!getInput("local_height", height))
        {
            throw RuntimeError("Missing input port [local_height]");
        }
        if (!getInput("global_radius", radius))
        {
            throw RuntimeError("Missing input port [global_radius]");
        }

        std::float_t width_list[] = {width/2, width/2, -1*width/2, -1*width/2};
        std::float_t height_list[] = {height/2, -1*height/2, -1*height/2, height/2};

        // サブスクライバの作成
        auto publisher = node_->create_publisher<geometry_msgs::msg::Polygon>("/local_costmap/footprint", 10);

        // poseへ数値設定
        geometry_msgs::msg::Polygon footprint;

        for(int i = 0; i < 4; i++){
            geometry_msgs::msg::Point32 p;
            p.x = width_list[i];
            p.y = height_list[i];
            p.z = 0.0;
            footprint.points.push_back(p);
        }

        max_publish_count_ = 10;
        publish_count_ = 0;

        while (publish_count_ < max_publish_count_ && rclcpp::ok()) {
            publisher->publish(footprint);
            publish_count_++;
            RCLCPP_INFO(node_->get_logger(), "Published Footprint %d/%d", publish_count_, max_publish_count_);

            // 適切な間隔を持たせるためにスリープ
            rclcpp::Rate(std::chrono::milliseconds(100)).sleep();  // 1Hzでパブリッシュ
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
    int publish_count_;
    int max_publish_count_;
};

#endif  // FOOTPRINT_SETTING_HPP
