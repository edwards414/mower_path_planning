cmake_minimum_required(VERSION 3.16)
project(mower_nav_smoother_oracle CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_velocity_smoother REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(oracle oracle.cpp)
# The production parameters: the oracle loads the file the robot runs with.
target_compile_definitions(oracle PRIVATE
  NAV2_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../mower_nav2/config/nav2_no_map_params.yaml")
ament_target_dependencies(oracle rclcpp rclcpp_lifecycle nav2_velocity_smoother nav2_util geometry_msgs
  lifecycle_msgs)
target_link_libraries(oracle nlohmann_json::nlohmann_json)
