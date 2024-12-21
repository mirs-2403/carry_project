#include "rclcpp/rclcpp.hpp"
#include "bt_actions/first_test.hpp"
#include "bt_actions/second_test.hpp"
#include "bt_actions/random_goal.hpp"
#include "bt_actions/check_call_service.hpp"
#include "bt_actions/print_message_action.hpp"
#include "bt_actions/subscribe_pose.hpp"
#include "bt_actions/is_called_service.hpp"
#include "bt_actions/compare_pose.hpp"
#include "bt_actions/pose_setting.hpp"
#include "bt_actions/footprint_setting.hpp"
//#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"
#include "nav2_behavior_tree/plugins/action/controller_cancel_node.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>

#define BT_XML "/home/oumuika/Documents/mirs2403/src/carry_project/carry/behavior_tree/test_7.xml"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 共有ノードの生成
    auto ros_node = rclcpp::Node::make_shared("bt_node");

    // Behavior Treeファクトリの作成
    BehaviorTreeFactory factory;

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", ros_node);

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
    
    BT::NodeBuilder node_builder = [&ros_node](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<SubscribePose>(name, config, ros_node);
    };

    BT::NodeBuilder node_builder2 = [&ros_node](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<PoseSetting>(name, config, ros_node);
    };

    BT::NodeBuilder node_builder3 = [&ros_node](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<FootprintSetting>(name, config, ros_node);
    };
    
    // Actionノードの登録
    factory.registerNodeType<FirstClientCall>("FirstClientCall");
    factory.registerNodeType<SecondClientCall>("SecondClientCall");
    factory.registerNodeType<RandomGoalGenerateAction>("GenerateRandomGoal");
    factory.registerNodeType<CheckCallService>("CheckCallService");
    factory.registerNodeType<IsCalledService>("IsCalledService");
    factory.registerNodeType<PrintMessageAction>("PrintMessage");
    factory.registerBuilder<SubscribePose>("SubscribePose", node_builder);
    factory.registerBuilder<ComparePose>("ComparePose", node_builder);
    factory.registerBuilder<PoseSetting>("PoseSetting", node_builder2);
    factory.registerBuilder<FootprintSetting>("FootprintSetting", node_builder3);

    std::vector<std::string> plugin_files;

    std::string plugin_directory = "/opt/ros/humble/lib";

    // 1. ディレクトリを探索し、"_bt_node.so" で終わるファイルを取得
    for (const auto &entry : fs::directory_iterator(plugin_directory)) {
        if (entry.is_regular_file()) {  // 通常ファイルのみ対象
        std::string file_path = entry.path().string();
        if (file_path.find("_bt_node.so") != std::string::npos) {  // "_bt_node.so" で終わるか確認
            plugin_files.push_back(file_path);
        }
        }
    }

    // 2. 各プラグインファイルをファクトリに登録
    for (const auto &plugin : plugin_files) {
        try {
        std::cout << "Loading: " << plugin << "\n";
        factory.registerFromPlugin(plugin);  // プラグインの登録
        } catch (const std::exception &e) {
        std::cerr << "Failed to load plugin: " << plugin << ", Error: " << e.what() << "\n";
        }
    }

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
