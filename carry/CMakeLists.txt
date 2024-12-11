cmake_minimum_required(VERSION 3.8)
project(carry)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(mirs_msgs REQUIRED)  # SimpleCommandサービス用

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(bt_executor src/main.cpp src/first_test.cpp src/second_test.cpp)
ament_target_dependencies(bt_executor
  rclcpp
  rclcpp_action
  mirs_msgs
  behaviortree_cpp_v3 
)

install(TARGETS
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
