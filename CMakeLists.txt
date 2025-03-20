cmake_minimum_required(VERSION 3.8)
project(scan_matching_odometry)

add_compile_options(-Wall -Wextra -Wpedantic -O3)

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (libpointmatcher REQUIRED)
find_package (tf2_ros REQUIRED)
find_package (rosbag2_transport REQUIRED)

add_executable (${PROJECT_NAME}_bag src/main_ros2bag.cpp src/matcher.cpp)
set_property (TARGET ${PROJECT_NAME}_bag PROPERTY CXX_STANDARD 20)
target_include_directories (${PROJECT_NAME}_bag PRIVATE ./include ${libpointmatcher_INCLUDE_DIRS})
target_link_libraries (${PROJECT_NAME}_bag ${libpointmatcher_LIBRARIES})
ament_target_dependencies (${PROJECT_NAME}_bag rclcpp sensor_msgs tf2_ros rosbag2_transport)

add_executable (${PROJECT_NAME}_node src/main_ros2node.cpp src/matcher.cpp)
set_property (TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD 20)
target_include_directories (${PROJECT_NAME}_node PRIVATE ./include ${libpointmatcher_INCLUDE_DIRS})
target_link_libraries (${PROJECT_NAME}_node ${libpointmatcher_LIBRARIES})
ament_target_dependencies (${PROJECT_NAME}_node rclcpp sensor_msgs tf2_ros)

install (TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_bag
  DESTINATION lib/${PROJECT_NAME}
)

install (FILES
  config/pm_config.yaml
  DESTINATION config/pm_config.yaml
)

ament_package()
