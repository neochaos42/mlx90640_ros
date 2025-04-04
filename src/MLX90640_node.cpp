#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include "mlx90640/MLX90640_I2C_Driver.h"
#include "mlx90640/MLX90640_API.h"
#include <vector>
#include <memory>

class MLX90640Node : public rclcpp::Node{
public:
    MLX90640Node(): Node("mlx90640_node"){
        // Declare parameters with default values
        this->declare_parameter<uint8_t>("camera.i2c_address", 0x33);
        this->declare_parameter<int>("camera.refresh_rate", 8);
        this->declare_parameter<float>("camera.emissivity", 0.95);
        this->declare_parameter<float>("camera.ambient_temperature", 23.0);

        // Get parameters from the parameter server
        this->get_parameter("camera.i2c_address", i2c_address_);
        this->get_parameter("camera.refresh_rate", refresh_rate_);
        this->get_parameter("camera.emissivity", emissivity_);
        this->get_parameter("camera.ambient_temperature", ambient_temperature_);

        // Log the retrieved parameters
        RCLCPP_INFO(this->get_logger(), "i2c_address: 0x%02x", i2c_address_);
        RCLCPP_INFO(this->get_logger(), "refresh_rate: %d Hz", refresh_rate_);
        RCLCPP_INFO(this->get_logger(), "emissivity: %f", emissivity_);
        RCLCPP_INFO(this->get_logger(), "ambient_temperature: %f Celsius", ambient_temperature_);

        // Validate parameters
        if (refresh_rate_ <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid refresh rate: %d", refresh_rate_);
            rclcpp::shutdown();
            return;
        }

        // Initialize the I2C communication
        RCLCPP_DEBUG(this->get_logger(), "Initializing I2C communication");
        if (MLX90640_I2CInit(this->get_logger()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize I2C communication");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully connected to I2C");

        // Initialize the MLX90640 sensor
        uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
        RCLCPP_DEBUG(this->get_logger(), "Dumping EEPROM data");
        int status = MLX90640_DumpEE(i2c_address_, eeData, this->get_logger());
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MLX90640 sensor, status: %d", status);
            MLX90640_I2CClose(this->get_logger());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully initialized MLX90640 sensor");

        // Extract parameters from EEPROM
        RCLCPP_DEBUG(this->get_logger(), "Extracting parameters from EEPROM data");
        status = MLX90640_ExtractParameters(eeData, &params_);
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract parameters from MLX90640 sensor, status: %d", status);
            MLX90640_I2CClose(this->get_logger());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully extracted parameters from MLX90640 sensor");

        // Set the refresh rate
        RCLCPP_DEBUG(this->get_logger(), "Setting refresh rate");
        status = MLX90640_SetRefreshRate(i2c_address_, refresh_rate_, this->get_logger());
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set refresh rate, status: %d", status);
            MLX90640_I2CClose(this->get_logger());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Refresh rate set to %d Hz", refresh_rate_);

        // Log the initialization of the publishers
        RCLCPP_DEBUG(this->get_logger(), "Initializing thermal image publisher");
        thermal_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);
        RCLCPP_INFO(this->get_logger(), "Thermal image publisher created");

        RCLCPP_DEBUG(this->get_logger(), "Initializing average temperature publisher");
        avg_temp_publisher_ = this->create_publisher<std_msgs::msg::Float32>("average_temperature", 10);
        RCLCPP_INFO(this->get_logger(), "Average temperature publisher created");

        // Create a timer to read and publish the thermal image periodically
        RCLCPP_DEBUG(this->get_logger(), "Creating timer");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / refresh_rate_), // Calculate interval based on refresh rate
            std::bind(&MLX90640Node::publishThermalImage, this));
        RCLCPP_INFO(this->get_logger(), "Timer created with interval: %d ms", 1000 / refresh_rate_);
    }

private:
    // Function to read and publish the thermal image
    void publishThermalImage()
    {
        RCLCPP_DEBUG(this->get_logger(), "Starting publishThermalImage function");

        // Read the frame data from the sensor
        uint16_t frameData[MLX90640_PIXEL_NUM];
        int status = MLX90640_GetFrameData(i2c_address_, frameData, this->get_logger());
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read frame data from MLX90640 sensor");
            return;
        }

        // Calculate the temperature values
        std::vector<float> temperatures(MLX90640_PIXEL_NUM);
        MLX90640_CalculateTo(frameData, &params_, emissivity_, ambient_temperature_, temperatures.data());

        // Create the thermal image message
        auto thermal_image_msg = std::make_unique<sensor_msgs::msg::Image>();
        thermal_image_msg->header.stamp = this->now();
        thermal_image_msg->header.frame_id = "thermal_camera";
        thermal_image_msg->height = MLX90640_LINE_NUM;
        thermal_image_msg->width = MLX90640_COLUMN_NUM;
        thermal_image_msg->encoding = "32FC1"; // 32-bit float, single channel
        thermal_image_msg->is_bigendian = false;
        thermal_image_msg->step = MLX90640_COLUMN_NUM * sizeof(float);
        thermal_image_msg->data.resize(MLX90640_PIXEL_NUM * sizeof(float));

        // Copy the temperature data into the image message
        memcpy(thermal_image_msg->data.data(), temperatures.data(), MLX90640_PIXEL_NUM * sizeof(float));

        // Publish the thermal image
        RCLCPP_DEBUG(this->get_logger(), "Publishing thermal image");
        thermal_image_publisher_->publish(std::move(thermal_image_msg));
        RCLCPP_INFO(this->get_logger(), "Thermal image published");

        // Calculate and publish the average temperature
        float avg_temp = MLX90640_GetTa(frameData, &params_);
        auto avg_temp_msg = std::make_unique<std_msgs::msg::Float32>();
        avg_temp_msg->data = avg_temp;
        RCLCPP_DEBUG(this->get_logger(), "Publishing average temperature");
        avg_temp_publisher_->publish(std::move(avg_temp_msg));
        RCLCPP_INFO(this->get_logger(), "Average temperature published: %f", avg_temp);
    }

    // ROS 2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_image_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr avg_temp_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // MLX90640 parameters
    paramsMLX90640 params_;

    // Parameters
    uint8_t i2c_address_;
    int refresh_rate_;
    float emissivity_;
    float ambient_temperature_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MLX90640Node>();
    RCLCPP_DEBUG(node->get_logger(), "Node initialized, starting spin");
    rclcpp::spin(node);
    RCLCPP_DEBUG(node->get_logger(), "Shutting down node");
    MLX90640_I2CClose(node->get_logger());
    rclcpp::shutdown();
    return 0;
}