cmake_minimum_required(VERSION 3.8)
project(clearpath_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)                      # DAYLAN
find_package(rosidl_default_generators REQUIRED)  # DAYLAN

# DAYLAN: Install Python Scripts
install(
    PROGRAMS
        src/clearpath_gz/waypoint_navigation.py
        src/clearpath_gz/gps_target.py
        src/clearpath_gz/target_detection.py
        src/clearpath_gz/target_tracking.py
        src/clearpath_gz/gps_target_race_track.py
        src/clearpath_gz/save_data.py
        src/clearpath_gz/path_following.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# DAYLAN: Add custom messages to package
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSTarget.msg"
  "msg/DataMessage.msg"
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
