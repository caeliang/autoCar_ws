cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 


include_directories(include)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(path_planner_node src/path_planner_node.cpp src/AStar.cpp src/global.cpp src/visualize.cpp)
ament_target_dependencies(path_planner_node rclcpp)
target_link_libraries(path_planner_node ${Python3_LIBRARIES})
option(USE_NATIVE_OPTIMIZATIONS "Enable aggressive compiler optimizations (O3/march=native)" ON)
if(USE_NATIVE_OPTIMIZATIONS)
  set(PERF_FLAGS -O3 -march=native -flto)
else()
  set(PERF_FLAGS -O3)
endif()
target_compile_options(path_planner_node PRIVATE ${PERF_FLAGS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Linker flags for LTO; target_link_options requires CMake >= 3.13
  if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.13")
    target_link_options(path_planner_node PRIVATE ${PERF_FLAGS})
  else()
    string(REPLACE ";" " " PERF_FLAGS_LINK "${PERF_FLAGS}")
    set_target_properties(path_planner_node PROPERTIES LINK_FLAGS "${PERF_FLAGS_LINK}")
  endif()
endif()
install(TARGETS path_planner_node DESTINATION lib/${PROJECT_NAME})


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
