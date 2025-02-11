#ifndef BACK_POSE_HPP_
#define BACK_POSE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class BackPose : public BT::SyncActionNode
{
public:
    BackPose(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
                 BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
                 BT::InputPort<std::float_t>("distance")};
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::msg::PoseStamped input_pose;
        if (!getInput<geometry_msgs::msg::PoseStamped>("pose", input_pose))
        {
            throw BT::RuntimeError("missing required input [pose]");
        }

        std::float_t distance; 
        if (!getInput("distance", distance))
        {
            throw RuntimeError("Missing input port [distance]");
        }

        geometry_msgs::msg::PoseStamped goal_pose = input_pose;

        // Calculate the new position 50cm behind the input pose
        tf2::Quaternion q(
            input_pose.pose.orientation.x,
            input_pose.pose.orientation.y,
            input_pose.pose.orientation.z,
            input_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        goal_pose.pose.position.x -= distance * cos(yaw);
        goal_pose.pose.position.y -= distance * sin(yaw);

        setOutput("goal", goal_pose);
        return BT::NodeStatus::SUCCESS;
    }
};

#endif  // BACK_POSE_HPP_