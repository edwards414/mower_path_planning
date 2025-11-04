cmake_minimum_required(VERSION 3.16)
project(lawer_qt_window)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for Qt6
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Qt6 dependencies
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui)

# Set Qt6 properties
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Source files
set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/models/MapDataModel.cpp
  src/models/PathDataModel.cpp
  src/models/ServiceModel.cpp
  src/controllers/MapController.cpp
  src/controllers/PathController.cpp
  src/controllers/ServiceController.cpp
    src/widgets/MapWidget.cpp
    src/widgets/ControlPanelWidget.cpp
    src/widgets/PathVisualizationWidget.cpp
    src/widgets/LogWidget.cpp
)

# Header files
set(HEADERS
  src/mainwindow.h
  src/models/MapDataModel.h
  src/models/PathDataModel.h
  src/models/ServiceModel.h
  src/controllers/MapController.h
  src/controllers/PathController.h
  src/controllers/ServiceController.h
  src/widgets/MapWidget.h
  src/widgets/ControlPanelWidget.h
  src/widgets/PathVisualizationWidget.h
  src/widgets/LogWidget.h
)

# UI files
set(UI_FILES
  src/mainwindow.ui
)

# Create executable
add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADERS} ${UI_FILES})

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  Qt6::Core
  Qt6::Widgets
  Qt6::Gui
)

# ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_srvs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

# Include directories
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
