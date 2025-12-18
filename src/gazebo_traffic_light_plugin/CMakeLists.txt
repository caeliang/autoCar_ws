cmake_minimum_required(VERSION 3.8)
project(gazebo_traffic_light_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build the auto traffic light plugin
add_library(auto_traffic_light_plugin SHARED
  auto_traffic_light_plugin.cpp
)

target_link_libraries(auto_traffic_light_plugin
  ${GAZEBO_LIBRARIES}
)

# Install targets
install(TARGETS
  auto_traffic_light_plugin
  LIBRARY DESTINATION lib
)

ament_package()
