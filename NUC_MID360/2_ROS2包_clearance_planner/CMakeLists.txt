cmake_minimum_required(VERSION 3.8)
project(medical_clearance_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(clearance_planner SHARED
  src/corner_geometry.cpp
  src/clearance_planner.cpp
)
target_include_directories(clearance_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(clearance_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core clearance_planner_plugin.xml)

add_library(parking_exemption_layer SHARED
  src/parking_exemption_geometry.cpp
  src/parking_exemption_layer.cpp
)
target_include_directories(parking_exemption_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parking_exemption_layer
  nav2_costmap_2d
  pluginlib
  rclcpp
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_corner_geometry test/test_corner_geometry.cpp)
  target_link_libraries(test_corner_geometry clearance_planner)
  ament_add_gtest(
    test_parking_exemption_geometry
    test/test_parking_exemption_geometry.cpp
    src/parking_exemption_geometry.cpp
  )
  target_include_directories(test_parking_exemption_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_parking_exemption_geometry nav2_costmap_2d)
endif()

install(TARGETS clearance_planner parking_exemption_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES clearance_planner_plugin.xml costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(clearance_planner parking_exemption_layer)
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
)
ament_package()
