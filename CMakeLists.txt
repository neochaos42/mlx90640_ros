cmake_minimum_required(VERSION 3.5)
project(mlx90640)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(${PROJECT_NAME}_node
  src/MLX90640_node.cpp
  src/MLX90640_I2C_Driver.cpp
  src/MLX90640_API.cpp
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs)

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
  DESTINATION share/${PROJECT_NAME}/config
)

# Ament package
ament_package()