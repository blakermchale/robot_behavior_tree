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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(robot_control_interfaces REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(runner_exe bt_runner)
add_executable(${runner_exe}
  src/main.cpp
)
set(runner_lib ${runner_exe}_core)
add_library(${runner_lib} SHARED
  src/bt_runner.cpp
)

set(dependencies
  rclcpp
  robot_control_interfaces
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_util
  nav2_bt_navigator
)

# Runner
ament_target_dependencies(${runner_exe}
  ${dependencies}
)
target_link_libraries(${runner_exe} ${runner_lib})
ament_target_dependencies(${runner_lib}
  ${dependencies}
)

# Plugins
add_library(robot_arm_takeoff_action_bt_node SHARED plugins/action/arm_takeoff_action.cpp)
list(APPEND plugin_libs robot_arm_takeoff_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${runner_lib}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${runner_exe}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  ${runner_lib}
)
ament_export_dependencies(${dependencies})
ament_package()
