#ifndef POSE_SETTING_HPP
#define POSE_SETTING_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"

using namespace BT;

class PoseSetting : public SyncActionNode
{
public:
    PoseSetting(const std::string& name, const NodeConfiguration& config, rclcpp::Node::SharedPtr ros_node)
    : SyncActionNode(name, config), node_(ros_node)
    {
        if (!node_)
        {
            throw std::runtime_error("Shared node is null.");
        }
    }

    static PortsList providedPorts(){
        return {
            InputPort<std::double_t>("position_x"),
            InputPort<std::double_t>("position_y"),
            InputPort<std::double_t>("orientation_theta"),
            OutputPort<geometry_msgs::msg::PoseStamped>("goal")
        };
    };

    NodeStatus tick() override
    {   

        // パラメータの取得
        std::double_t x;
        std::double_t y;
        std::double_t theta;

        if (!getInput("position_x", x))
        {
            throw RuntimeError("Missing input port [position_x]");
        }
        if (!getInput("position_y", y))
        {
            throw RuntimeError("Missing input port [position_y]");
        }
        if (!getInput("orientation_theta", theta))
        {
            throw RuntimeError("Missing input port [orientation_theta]");
        }
        
        // poseへ数値設定
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = rclcpp::Clock().now();

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        // 姿勢を設定 (クォータニオン)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();


        setOutput("goal", pose);

        return BT::NodeStatus::SUCCESS;
    };



private:
    rclcpp::Node::SharedPtr node_; // ROS 2ノード
};

#endif  // POSE_SETTING_HPP
