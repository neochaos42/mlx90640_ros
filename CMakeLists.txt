cmake_minimum_required(VERSION 3.5)
project(mlx90640)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Add executable
add_executable(${PROJECT_NAME}_node
  src/MLX90640_node.cpp
  src/MLX90640_I2C_Driver.cpp
  src/MLX90640_API.cpp
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs)

# Install executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()