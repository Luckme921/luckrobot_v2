cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/motion_freshness_guard.py
  DESTINATION lib/${PROJECT_NAME} RENAME motion_freshness_guard)
install(FILES scripts/motion_freshness_core.py DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  rviz_config
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vslam_odom_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vslam_odom_guard
)

install(PROGRAMS
  scripts/vslam_stereo_monotonic_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vslam_stereo_monotonic_gate
)

install(PROGRAMS
  scripts/depth_height_scan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_height_scan
)

install(PROGRAMS
  scripts/scan_confirmation_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_confirmation_filter
)

install(PROGRAMS
  scripts/wheel_yaw_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_yaw_guard
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

ament_package()
