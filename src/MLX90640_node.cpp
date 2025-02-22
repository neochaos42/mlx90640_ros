#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include "mlx90640/MLX90640_I2C_Driver.h"
#include "mlx90640/MLX90640_API.h"
#include <vector>
#include <memory>

class MLX90640Node : public rclcpp::Node
{
public:
    MLX90640Node()
        : Node("mlx90640_node")
    {
        // Declare parameters
        this->declare_parameter<uint8_t>("camera.i2c_address", 0x33);
        this->declare_parameter<int>("camera.refresh_rate", 8);
        this->declare_parameter<float>("camera.emissivity", 0.95);
        this->declare_parameter<float>("camera.ambient_temperature", 23.0);

        // Get parameters
        this->get_parameter("camera.i2c_address", i2c_address_);
        this->get_parameter("camera.refresh_rate", refresh_rate_);
        this->get_parameter("camera.emissivity", emissivity_);
        this->get_parameter("camera.ambient_temperature", ambient_temperature_);

        // Initialize the I2C communication
        MLX90640_I2CInit();

        // Initialize the MLX90640 sensor
        uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
        int status = MLX90640_DumpEE(i2c_address_, eeData);
        if (status != MLX90640_NO_ERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MLX90640 sensor");
            return;
        }

        // Extract parameters from EEPROM
        status = MLX90640_ExtractParameters(eeData, &params_);
        if (status != MLX90640_NO_ERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract parameters from MLX90640 sensor");
            return;
        }

        // Set the refresh rate
        MLX90640_SetRefreshRate(i2c_address_, refresh_rate_);

        // Create a publisher for the thermal image
        thermal_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);

        // Create a publisher for the average temperature
        avg_temp_publisher_ = this->create_publisher<std_msgs::msg::Float32>("average_temperature", 10);

        // Create a timer to read and publish the thermal image periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / refresh_rate_), // Calculate interval based on refresh rate
            std::bind(&MLX90640Node::publishThermalImage, this));
    }

private:
    void publishThermalImage()
    {
        // Read the frame data from the sensor
        uint16_t frameData[MLX90640_PIXEL_NUM];
        int status = MLX90640_GetFrameData(i2c_address_, frameData);
        if (status != MLX90640_NO_ERROR)
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
        thermal_image_publisher_->publish(std::move(thermal_image_msg));

        // Calculate and publish the average temperature
        float avg_temp = MLX90640_GetTa(frameData, &params_);
        auto avg_temp_msg = std::make_unique<std_msgs::msg::Float32>();
        avg_temp_msg->data = avg_temp;
        avg_temp_publisher_->publish(std::move(avg_temp_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_image_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr avg_temp_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    paramsMLX90640 params_; // Store the parameters as a member variable

    // Parameters
    uint8_t i2c_address_;
    int refresh_rate_;
    float emissivity_;
    float ambient_temperature_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MLX90640Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}