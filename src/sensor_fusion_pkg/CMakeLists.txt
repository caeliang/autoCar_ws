cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# --------- Kalman Filter Library ---------
add_library(kalman_filter
    src/kalman_filter.cpp
)

target_include_directories(kalman_filter PUBLIC include)
target_link_libraries(kalman_filter Eigen3::Eigen)

# --------- Node Executable ---------
add_executable(sensor_fusion_node
    src/sensor_fusion_node.cpp
)

target_link_libraries(sensor_fusion_node
    kalman_filter
)

ament_target_dependencies(sensor_fusion_node
    rclcpp
    sensor_msgs
    nav_msgs
)

# --------- Install ---------
install(TARGETS
  kalman_filter
  sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
install(
  PROGRAMS scripts/log_and_plot.py
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
