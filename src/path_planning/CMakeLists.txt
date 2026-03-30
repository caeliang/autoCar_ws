cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(waypoint_utils
  src/waypoint_io.cpp
  src/waypoint_visualizer.cpp
)
ament_target_dependencies(waypoint_utils rclcpp visualization_msgs geometry_msgs)

add_executable(waypoint_manager src/waypoint_manager_node.cpp)
target_link_libraries(waypoint_manager waypoint_utils)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs nav_msgs std_msgs)

add_executable(pure_pursuit src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit rclcpp nav_msgs geometry_msgs)

add_executable(lane_path_follower src/lane_path_follower.cpp)
ament_target_dependencies(lane_path_follower rclcpp geometry_msgs nav_msgs)

add_executable(waypoint_lane_fusion src/waypoint_lane_fusion_node.cpp)
ament_target_dependencies(waypoint_lane_fusion rclcpp geometry_msgs nav_msgs)

install(TARGETS
  waypoint_manager
  pure_pursuit
  lane_path_follower
  waypoint_lane_fusion
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_recorder.py
  scripts/waypoint_visualizer.py
  scripts/road_network.py
  scripts/route_generator.py
  DESTINATION lib/${PROJECT_NAME}
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
