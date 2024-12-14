cmake_minimum_required(VERSION 3.8)
project(carry)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(mirs_msgs REQUIRED)

find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(bt_executor 
  src/main.cpp
  src/first_test.cpp
  src/second_test.cpp
  src/random_goal.cpp
  src/check_call_service.cpp
)
ament_target_dependencies(bt_executor
  rclcpp
  rclcpp_action
  mirs_msgs
  #behaviortree_cpp
  behaviortree_cpp_v3
  nav2_util
  nav_msgs
  nav2_msgs
  nav2_bt_navigator
  geometry_msgs
  tf2
  tf2_ros
  nav2_behavior_tree
)

add_executable(test_client src/test_client.cpp)
ament_target_dependencies(test_client
  rclcpp
  mirs_msgs
)

install(TARGETS
  bt_executor
  test_client
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
