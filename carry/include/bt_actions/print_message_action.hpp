#ifndef PRINT_MESSAGE_ACTION_HPP
#define PRINT_MESSAGE_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include <string>
#include <iostream>

class PrintMessageAction : public BT::SyncActionNode
{
public:
  // コンストラクタ
  PrintMessageAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  // 必須メソッド: 入出力ポートの定義
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message")};
  }

  // tick()メソッド: アクションの本体
  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("Missing required input [message]: ", msg.error());
    }

    std::cout << "Received message: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

#endif // PRINT_MESSAGE_ACTION_HPP
