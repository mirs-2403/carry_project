#include "rclcpp/rclcpp.hpp"
#include "bt_actions/is_current_pose.hpp"

using namespace BT;

IsCurrentPose::IsCurrentPose(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("check_aruco_marker_node");
    subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco_marker", 10, std::bind(&IsCurrentPose::markerCallback, this, std::placeholders::_1));
}

NodeStatus IsCurrentPose::tick()
{
    rclcpp::spin_some(node_);

    if(is_current_pose){
        return NodeStatus::SUCCESS;
    }else{
        return NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

PortsList IsCurrentPose::providedPorts()
{
    return PortsList();  // 必要に応じてポートを定義
}

void IsCurrentPose::markerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    
    

}


