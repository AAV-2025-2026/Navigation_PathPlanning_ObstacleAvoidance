cmake_minimum_required(VERSION 3.8)
project(navigation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(path_planning_module src/navigation_system.cpp)

# Dependencies
ament_target_dependencies(path_planning_module
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
)

# Link PCL libraries
target_link_libraries(path_planning_module
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS
  path_planning_module
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
