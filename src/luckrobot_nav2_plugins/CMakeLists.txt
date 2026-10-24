cmake_minimum_required(VERSION 3.8)
project(luckrobot_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/luck_controller.cpp
  src/luck_planner.cpp
  src/persistent_camera_layer.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core luckrobot_nav2_plugins.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d camera_layer_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES luckrobot_nav2_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib
  rclcpp rclcpp_lifecycle sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_path_rounding ../../tools/test_path_rounding.cpp)
  target_include_directories(test_path_rounding PRIVATE include)
  target_compile_features(test_path_rounding PRIVATE cxx_std_17)
  add_test(NAME path_rounding COMMAND test_path_rounding)
  add_executable(test_planner_rounding ../../tools/test_planner_rounding.cpp)
  target_link_libraries(test_planner_rounding ${PROJECT_NAME})
  ament_target_dependencies(test_planner_rounding nav2_costmap_2d)
  add_test(NAME planner_rounding COMMAND test_planner_rounding)
  set_tests_properties(planner_rounding PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=191;ROS_LOCALHOST_ONLY=1" TIMEOUT 30)
  add_executable(test_corner_speed ../../tools/test_corner_speed.cpp)
  target_include_directories(test_corner_speed PRIVATE include)
  target_compile_features(test_corner_speed PRIVATE cxx_std_17)
  add_test(NAME corner_speed COMMAND test_corner_speed)
  add_executable(test_motion_prediction ../../tools/test_motion_prediction.cpp)
  target_include_directories(test_motion_prediction PRIVATE include)
  target_compile_features(test_motion_prediction PRIVATE cxx_std_17)
  add_test(NAME motion_prediction COMMAND test_motion_prediction)
  add_executable(test_controller_prediction ../../tools/test_controller_prediction.cpp)
  target_link_libraries(test_controller_prediction ${PROJECT_NAME})
  ament_target_dependencies(test_controller_prediction nav2_costmap_2d)
  add_test(NAME controller_prediction COMMAND test_controller_prediction)
  set_tests_properties(controller_prediction PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=191;ROS_LOCALHOST_ONLY=1" TIMEOUT 30)
  add_executable(test_camera_layer_resize ../../tools/test_camera_layer_resize.cpp)
  target_link_libraries(test_camera_layer_resize ${PROJECT_NAME})
  ament_target_dependencies(test_camera_layer_resize nav2_costmap_2d pluginlib)
endif()

ament_package()
