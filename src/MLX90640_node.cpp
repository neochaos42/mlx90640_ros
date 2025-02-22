#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
        // Initialize the I2C communication
        MLX90640_I2CInit();

        // Initialize the MLX90640 sensor
        uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
        int status = MLX90640_DumpEE(0x33, eeData);
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

        // Set the refresh rate (e.g., 8Hz)
        MLX90640_SetRefreshRate(0x33, 0x03);

        // Create a publisher for the thermal image
        thermal_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);

        // Create a timer to read and publish the thermal image periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(125), // 8Hz
            std::bind(&MLX90640Node::publishThermalImage, this));
    }

private:
    void publishThermalImage()
    {
        // Read the frame data from the sensor
        uint16_t frameData[MLX90640_PIXEL_NUM];
        int status = MLX90640_GetFrameData(0x33, frameData);
        if (status != MLX90640_NO_ERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read frame data from MLX90640 sensor");
            return;
        }

        // Calculate the temperature values
        std::vector<float> temperatures(MLX90640_PIXEL_NUM);
        MLX90640_CalculateTo(frameData, &params_, 0.95, 23.0, temperatures.data());

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
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    paramsMLX90640 params_; // Store the parameters as a member variable
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MLX90640Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}