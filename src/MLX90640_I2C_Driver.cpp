#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c.h>
#include <rclcpp/rclcpp.hpp>
#include "mlx90640/MLX90640_I2C_Driver.h"

#define I2C_DEVICE "/dev/i2c-1"  // Change this to the appropriate I2C bus on your Jetson Orin

static int i2c_fd = -1;

// Function to initialize the I2C communication
int MLX90640_I2CInit(rclcpp::Logger logger) {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }

    RCLCPP_INFO(logger, "Opening I2C device: %s", I2C_DEVICE);
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        RCLCPP_ERROR(logger, "Failed to open I2C device: %s", strerror(errno));
        return -1;
    }
    RCLCPP_INFO(logger, "I2C device opened successfully");
    return 0;
}

// Function to read data from the MLX90640 sensor
int MLX90640_I2CRead(rclcpp::Logger logger, uint8_t deviceAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data) {
    uint8_t buf[2];
    int ret;

    if (i2c_fd < 0) {
        RCLCPP_ERROR(logger, "I2C device not initialized");
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, deviceAddr) < 0) {
        RCLCPP_ERROR(logger, "Failed to set I2C device address: %s", strerror(errno));
        return -1;
    }

    buf[0] = (startAddress >> 8) & 0xFF;
    buf[1] = startAddress & 0xFF;

    if (write(i2c_fd, buf, 2) != 2) {
        RCLCPP_ERROR(logger, "Failed to write start address: %s", strerror(errno));
        return -1;
    }

    ret = read(i2c_fd, data, nMemAddressRead * 2);
    if (ret != nMemAddressRead * 2) {
        RCLCPP_ERROR(logger, "Failed to read data: %s", strerror(errno));
        return -1;
    }

    return 0;
}

// Function to write data to the MLX90640 sensor
int MLX90640_I2CWrite(rclcpp::Logger logger, uint8_t deviceAddr, uint16_t writeAddress, uint16_t data) {
    uint8_t buf[4];
    int ret;

    if (i2c_fd < 0) {
        RCLCPP_ERROR(logger, "I2C device not initialized");
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, deviceAddr) < 0) {
        RCLCPP_ERROR(logger, "Failed to set I2C device address: %s", strerror(errno));
        return -1;
    }

    buf[0] = (writeAddress >> 8) & 0xFF;
    buf[1] = writeAddress & 0xFF;
    buf[2] = (data >> 8) & 0xFF;
    buf[3] = data & 0xFF;

    ret = write(i2c_fd, buf, 4);
    if (ret != 4) {
        RCLCPP_ERROR(logger, "Failed to write data: %s", strerror(errno));
        return -1;
    }

    return 0;
}

// Function to perform a general I2C reset
int MLX90640_I2CGeneralReset(rclcpp::Logger logger) {
    uint8_t buf[2] = {0x00, 0x06};

    if (i2c_fd < 0) {
        RCLCPP_ERROR(logger, "I2C device not initialized");
        return -1;
    }

    RCLCPP_INFO(logger, "Setting I2C general call address");
    if (ioctl(i2c_fd, I2C_SLAVE, 0x00) < 0) {
        RCLCPP_ERROR(logger, "Failed to set I2C general call address: %s", strerror(errno));
        return -1;
    }

    RCLCPP_INFO(logger, "Sending I2C general call reset command");
    if (write(i2c_fd, buf, 2) != 2) {
        RCLCPP_ERROR(logger, "Failed to send I2C general call reset: %s", strerror(errno));
        return -1;
    }

    RCLCPP_INFO(logger, "I2C general call reset command sent successfully");
    return 0;
}

// Function to close the I2C communication
int MLX90640_I2CClose(rclcpp::Logger logger) {
    if (i2c_fd >= 0) {
        RCLCPP_INFO(logger, "Closing I2C device");
        if (close(i2c_fd) == 0) {
            RCLCPP_INFO(logger, "I2C device closed successfully");
            i2c_fd = -1;
            return 0;
        } else {
            RCLCPP_ERROR(logger, "Failed to close I2C device: %s", strerror(errno));
            return -1;
        }
    } else {
        RCLCPP_ERROR(logger, "I2C device was not open");
        return -1;
    }
}
