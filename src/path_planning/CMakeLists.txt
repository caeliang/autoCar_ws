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
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(perception REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/DenseWaypoints.msg"
#   DEPENDENCIES geometry_msgs std_msgs
# )

include_directories(include)

add_library(waypoint_utils
  src/core/waypoint_utils.cpp
  src/core/waypoint_io.cpp
  src/core/waypoint_visualizer.cpp
)
ament_target_dependencies(waypoint_utils rclcpp visualization_msgs geometry_msgs)

add_executable(waypoint_tracker src/nodes/waypoint_tracker.cpp)
target_link_libraries(waypoint_tracker waypoint_utils)
ament_target_dependencies(waypoint_tracker rclcpp nav_msgs geometry_msgs std_msgs)

add_executable(car_index_finder src/nodes/car_index_finder.cpp)
ament_target_dependencies(car_index_finder rclcpp geometry_msgs)

add_executable(waypoint_manager src/nodes/waypoint_manager_node.cpp)
target_link_libraries(waypoint_manager waypoint_utils)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs nav_msgs std_msgs std_srvs)

add_executable(lane_path_follower src/nodes/lane_path_follower.cpp)
ament_target_dependencies(lane_path_follower rclcpp geometry_msgs nav_msgs visualization_msgs)

# a_star_planner is only used as a library, not a standalone executable
# add_executable(a_star_planner src/a_star_planner.cpp)
# ament_target_dependencies(a_star_planner rclcpp)

add_executable(path_planning_main src/algorithms/path_planning_main.cpp)
ament_target_dependencies(path_planning_main rclcpp geometry_msgs nav_msgs)

install(TARGETS
  waypoint_tracker
  car_index_finder
  waypoint_manager
  lane_path_follower
  path_planning_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/visualization/waypoint_recorder.py
  scripts/visualization/waypoint_visualizer.py
  scripts/path_planning/road_network.py
  scripts/path_planning/generate_full_map.py
  scripts/path_planning/generate_route.py
  scripts/path_planning/route_recorder
  scripts/path_planning/route_recorder.py
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
