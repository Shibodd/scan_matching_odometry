cmake_minimum_required(VERSION 3.8)
project(scan_matching_odometry)

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (libpointmatcher REQUIRED)
find_package (tf2_ros REQUIRED)

add_executable (${PROJECT_NAME} src/main.cpp)
set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
target_include_directories (${PROJECT_NAME} PRIVATE ${libpointmatcher_INCLUDE_DIRS})
target_link_libraries (${PROJECT_NAME} ${libpointmatcher_LIBRARIES})
ament_target_dependencies (${PROJECT_NAME} rclcpp sensor_msgs tf2_ros)

install (TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
