cmake_minimum_required(VERSION 3.8)
project(my_robot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Linux executable shebangs must never retain Windows CRLF, even when copied
# from a Windows working tree without Git checkout normalization.
configure_file(scripts/map_session_manager.py
  ${CMAKE_CURRENT_BINARY_DIR}/map_session_manager @ONLY NEWLINE_STYLE UNIX)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/map_session_manager
  DESTINATION lib/${PROJECT_NAME} RENAME map_session_manager)
install(FILES scripts/map_versions.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME map_session_regression
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_map_session.py)
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
