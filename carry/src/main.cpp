#include "rclcpp/rclcpp.hpp"
#include "bt_actions/first_test.hpp"
#include "bt_actions/second_test.hpp"
#include "bt_actions/random_goal.hpp"
#include "bt_actions/check_call_service.hpp"
#include "bt_actions/print_message_action.hpp"
//#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
//#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define BT_XML "/home/oumuika/Documents/mirs2403/src/carry_project/carry/behavior_tree/test_4.xml"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 共有ノードの生成
    auto ros_node = rclcpp::Node::make_shared("bt_node");

    // Behavior Treeファクトリの作成
    BehaviorTreeFactory factory;

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", ros_node);

    /*
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map"; // 目標座標系（例: "map"）
    goal.header.stamp = rclcpp::Clock().now(); // 現在時刻

    // 位置を設定
    goal.pose.position.x = 1.0;  // 目標のX座標
    goal.pose.position.y = 2.0;  // 目標のY座標
    goal.pose.position.z = 0.0;  // Zは通常0（2Dナビゲーションの場合）

    // 姿勢を設定 (クォータニオン)
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    blackboard->set("goal", goal);

    // 0.1秒（100ミリ秒）
    std::chrono::milliseconds bt_loop_duration(100); 
    // Blackboardに設定
    blackboard->set("bt_loop_duration", bt_loop_duration);
    // 5秒をミリ秒単位に変換
    std::chrono::milliseconds server_timeout(5000);  // 5秒 = 5000ミリ秒
    // Blackboardに設定
    blackboard->set("server_timeout", server_timeout);
    // サービス待機タイムアウトを10秒に設定
    std::chrono::milliseconds wait_for_service_timeout(10000);
    // Blackboardに設定
    blackboard->set("wait_for_service_timeout", wait_for_service_timeout);
    */

    // Actionノードの登録
    factory.registerNodeType<FirstClientCall>("FirstClientCall");
    factory.registerNodeType<SecondClientCall>("SecondClientCall");
    factory.registerNodeType<RandomGoalGenerateAction>("GenerateRandomGoal");
    factory.registerNodeType<CheckCallService>("CheckCallService");
    factory.registerNodeType<PrintMessageAction>("PrintMessage");
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_navigate_to_pose_action_bt_node.so");

    // XMLファイルからツリーを読み込む
    auto tree = factory.createTreeFromFile(BT_XML, blackboard);

    // get the tree and poll status updates.
    //const unsigned port = 5555;
    //BT::Groot2Publisher publisher(tree, port);
    BT::PublisherZMQ publisher_zmq(tree);

    // Behavior Treeの実行ループ
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickRoot();
        rclcpp::spin_some(ros_node);
    }

    //rclcpp::shutdown();
    return 0;
}
