cmake_minimum_required(VERSION 3.8)
project(mower_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find libserial using PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Hardware interface library
add_library(
  mower_controller
  SHARED
  src/mower_system.cpp
  src/Stm_Comms.cpp
  src/wheel.cpp
)

target_include_directories(
  mower_controller
  PRIVATE
  hardware/include
  ${LIBSERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(
  mower_controller
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link libserial
target_link_libraries(
  mower_controller
  ${LIBSERIAL_LIBRARIES}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# Install library
install(
  TARGETS mower_controller
  DESTINATION lib
)

# Install header files
install(
  DIRECTORY hardware/include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  controllers
  DESTINATION share/${PROJECT_NAME}/
)

# Install robot hardware description
install(FILES
  robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(
  include
)
ament_export_libraries(
  mower_controller
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
