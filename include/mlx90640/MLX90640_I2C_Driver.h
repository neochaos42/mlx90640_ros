/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _MLX90640_I2C_Driver_H_
    #define _MLX90640_I2C_Driver_H_

    #include <cstdint>
    #include <rclcpp/rclcpp.hpp>
    #include "MLX90640_API.h"

    int MLX90640_I2CInit(rclcpp::Logger logger);
    int MLX90640_I2CGeneralReset(rclcpp::Logger logger);
    int MLX90640_I2CRead(rclcpp::Logger logger, uint8_t deviceAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
    int MLX90640_I2CWrite(rclcpp::Logger logger, uint8_t deviceAddr, uint16_t writeAddress, uint16_t data);
    int MLX90640_I2CClose(rclcpp::Logger logger);

#endif