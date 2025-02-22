# MLX90640 ROS Package

This ROS package publishes data from the MLX90640 thermal camera.

## Overview

The `mlx90640` package provides a ROS node that interfaces with the MLX90640 thermal camera, reads thermal data, and publishes it as ROS messages.

## Installation

1. Clone the repository into your ROS workspace:
    ```sh
    cd ~/dev_ws/src
    git clone <repository_url>
    ```

2. Install dependencies:
    ```sh
    cd ~/dev_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the package:
    ```sh
    colcon build
    ```

4. Source the workspace:
    ```sh
    source ~/dev_ws/install/setup.bash
    ```

## Usage

### Running the Node

To run the `mlx90640_node`, use the provided launch file:
```sh
ros2 launch mlx90640 mlx90640.launch.py
```

### Parameters

The node can be configured using parameters specified in the `config/mlx906040_config.yaml` file:

- `camera.i2c_address`: I2C address of the MLX90640 camera (default: `0x33`)
- `camera.refresh_rate`: Refresh rate of the camera in Hz (default: `8`)
- `camera.emissivity`: Emissivity of the object being measured (default: `0.95`)
- `camera.ambient_temperature`: Ambient temperature in Celsius (default: `23.0`)

### Topics

The node publishes the following topics:

- `/thermal_image` (`sensor_msgs/msg/Image`): The thermal image data from the MLX90640 camera.

## License

This project is licensed under the Apache License 2.0 - see the `LICENSE` file for details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.
