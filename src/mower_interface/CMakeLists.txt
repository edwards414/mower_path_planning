cmake_minimum_required(VERSION 3.8)
project(mower_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneMap.msg"
  "srv/GetZoneList.srv"
  "srv/ZoneMapList.srv"
  "srv/ZoneExecPath.srv"
  "srv/ChannelPathList.srv"
  "srv/ChennalPathList.srv"
  "srv/ChannelRoute.srv"
  "srv/ImportImageMask.srv"
  "srv/ZoneSequence.srv"
  "srv/EditZone.srv"
  "action/Waypoint.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs visualization_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  action_msgs
)
ament_package()
