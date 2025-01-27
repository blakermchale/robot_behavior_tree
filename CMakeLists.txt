cmake_minimum_required(VERSION 3.5)
project(robot_behavior_tree)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  robot_control_interfaces
  behaviortree_cpp
  behaviortree_ros2
)

######################################################
# Simple example showing how to use and customize the BtExecutionServer
add_executable(bt_runner src/bt_runner.cpp)
ament_target_dependencies(bt_runner ${dependencies})

# Plugins
add_library(robot_arm_takeoff_action_bt_node SHARED plugins/action/arm_takeoff_action.cpp)
list(APPEND plugin_libs robot_arm_takeoff_action_bt_node)
add_library(robot_land_action_bt_node SHARED plugins/action/land_action.cpp)
list(APPEND plugin_libs robot_land_action_bt_node)
add_library(robot_go_waypoint_action_bt_node SHARED plugins/action/go_waypoint_action.cpp)
list(APPEND plugin_libs robot_go_waypoint_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION share/${PROJECT_NAME}/plugins
  LIBRARY DESTINATION share/${PROJECT_NAME}/plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/plugins
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY trees launch config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
