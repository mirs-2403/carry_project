#include "rclcpp/rclcpp.hpp"
#include "mirs_msgs/srv/number_command.hpp"  // mirs_msgs/srv/NumberCommandをインクルード

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ノードの作成
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_client");
  
  // サービスクライアントの作成
  rclcpp::Client<mirs_msgs::srv::NumberCommand>::SharedPtr client =
      node->create_client<mirs_msgs::srv::NumberCommand>("test_service");

  // リクエストメッセージを作成
  auto request = std::make_shared<mirs_msgs::srv::NumberCommand::Request>();
  request->parameter = 2.0;  // 送信する値

  // サービスが起動するまで待機
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // 非同期リクエストを送信
  auto result = client->async_send_request(request);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Send Request");

  // リクエストの完了を待機し、結果をチェック
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    // レスポンスが正常に受信できた場合
    if (result.get()->success) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call succeeded: Success = true");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed: Success = false");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}