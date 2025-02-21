cmake_minimum_required(VERSION 3.8)
project(wpf_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(wpf_msgs REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  nav2_behavior_tree
  wpf_msgs
)

add_library(analyze_logs_action_bt_node SHARED
  src/analyze_logs_action.cpp
)

target_compile_definitions(analyze_logs_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS analyze_logs_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_target_dependencies(analyze_logs_action_bt_node
  ${dependencies}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(analyze_logs_action_bt_node)
ament_export_dependencies(${dependencies})

ament_package()
