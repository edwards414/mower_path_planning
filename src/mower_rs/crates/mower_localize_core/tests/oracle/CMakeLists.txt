# Builds the oracle harness against the installed robot_localization. Run it
# inside the mower-jazzy-test:tools container; see run_oracle.sh.
cmake_minimum_required(VERSION 3.16)
project(mower_localize_oracle CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(oracle oracle.cpp)
target_include_directories(oracle PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(oracle
  rclcpp robot_localization nav_msgs sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs diagnostic_updater diagnostic_msgs
  message_filters geographic_msgs angles std_srvs)
target_link_libraries(oracle /opt/ros/jazzy/lib/librl_lib.so ${GeographicLib_LIBRARIES})
