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
#include "mlx90640/MLX90640_I2C_Driver.h"

#define I2C_DEVICE "/dev/i2c-1"  // Change this to the appropriate I2C bus on your Jetson Orin

static int i2c_fd = -1;

// Function to initialize the I2C communication
void MLX90640_I2CInit() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }

    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        exit(1);
    }
}

// Function to read data from the MLX90640 sensor
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data) {
    uint8_t buf[2];
    int ret;

    if (i2c_fd < 0) {
        std::cerr << "I2C device not initialized" << std::endl;
        return -1;
    }

    // Set the slave address
    if (ioctl(i2c_fd, I2C_SLAVE, slaveAddr) < 0) {
        perror("Failed to set I2C slave address");
        return -1;
    }

    // Prepare the start address
    buf[0] = (startAddress >> 8) & 0xFF;  // MSB
    buf[1] = startAddress & 0xFF;         // LSB

    // Write the start address
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Failed to write start address");
        return -1;
    }

    // Read the data
    ret = read(i2c_fd, data, nMemAddressRead * 2);
    if (ret != nMemAddressRead * 2) {
        perror("Failed to read data");
        return -1;
    }

    return 0;
}

// Function to write data to the MLX90640 sensor
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    uint8_t buf[4];
    int ret;

    if (i2c_fd < 0) {
        std::cerr << "I2C device not initialized" << std::endl;
        return -1;
    }

    // Set the slave address
    if (ioctl(i2c_fd, I2C_SLAVE, slaveAddr) < 0) {
        perror("Failed to set I2C slave address");
        return -1;
    }

    // Prepare the write address and data
    buf[0] = (writeAddress >> 8) & 0xFF;  // MSB of address
    buf[1] = writeAddress & 0xFF;         // LSB of address
    buf[2] = (data >> 8) & 0xFF;          // MSB of data
    buf[3] = data & 0xFF;                 // LSB of data

    // Write the data
    ret = write(i2c_fd, buf, 4);
    if (ret != 4) {
        perror("Failed to write data");
        return -1;
    }

    return 0;
}

// Function to perform a general I2C reset
int MLX90640_I2CGeneralReset() {
    uint8_t buf[2] = {0x00, 0x06};  // I2C general call reset command

    if (i2c_fd < 0) {
        std::cerr << "I2C device not initialized" << std::endl;
        return -1;
    }

    // Set the slave address to 0x00 (general call address)
    if (ioctl(i2c_fd, I2C_SLAVE, 0x00) < 0) {
        perror("Failed to set I2C general call address");
        return -1;
    }

    // Send the reset command
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Failed to send I2C general call reset");
        return -1;
    }

    return 0;
}