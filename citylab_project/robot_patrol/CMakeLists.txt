cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(std_msgs REQUIRED)

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
add_executable(patrol_exe src/patrol.cpp)
ament_target_dependencies(patrol_exe rclcpp geometry_msgs nav_msgs sensor_msgs)
install(TARGETS patrol_exe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
add_executable(patrol_real_exe src/patrol_real_robot.cpp)
ament_target_dependencies(patrol_real_exe rclcpp geometry_msgs nav_msgs sensor_msgs)
install(TARGETS patrol_real_exe DESTINATION lib/${PROJECT_NAME})

add_executable(scan_size_exe src/scan_size.cpp)
ament_target_dependencies(scan_size_exe rclcpp geometry_msgs nav_msgs sensor_msgs)
install(TARGETS scan_size_exe DESTINATION lib/${PROJECT_NAME})