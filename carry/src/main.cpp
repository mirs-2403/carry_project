#include "rclcpp/rclcpp.hpp"
#include "bt_actions/first_test.hpp"
#include "bt_actions/second_test.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#define BT_XML "/home/oumuika/Documents/mirs2403/src/carry_project/carry/behavior_tree/test.xml"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 共有ノードの生成
    //auto shared_node = create_shared_node();

    // Behavior Treeファクトリの作成
    BehaviorTreeFactory factory;

    // Actionノードの登録
    factory.registerNodeType<FirstClientCall>("FirstClientCall");
    factory.registerNodeType<SecondClientCall>("SecondClientCall");

    // XMLファイルからツリーを読み込む
    auto tree = factory.createTreeFromFile(BT_XML);

    PublisherZMQ publisher_zmq(tree);

    // Behavior Treeの実行ループ
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickRoot();
        //rclcpp::spin_some(shared_node);
    }

    //rclcpp::shutdown();
    return 0;
}
