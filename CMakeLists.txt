cmake_minimum_required(VERSION 3.5)
project(mlx90640)

# Default to C++14 and C99
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Add executable for the node
add_executable(mlx90640_node 
  src/MLX90640_node.cpp 
  src/MLX90640_API.c 
  src/MLX90640_I2C_Driver.c
)

# Link the node with required libraries
ament_target_dependencies(mlx90640_node 
  rclcpp 
  std_msgs 
  sensor_msgs
)

# Install the node executable
install(TARGETS
  mlx90640_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

# Install include files
install(DIRECTORY
  include/
  DESTINATION include/)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

# Install LICENSE
install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME})

# Finalize the package configuration
ament_package()