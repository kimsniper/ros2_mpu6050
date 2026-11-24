/*
 * Copyright (c) 2022, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros2_mpu6050/mpu6050_hal.h"

/* User header files */
#include <iostream>

Mpu6050Hal::Mpu6050Hal(const std::string& device, const uint8_t i2c_address) 
{ 
    /* User implementation here */

    fd_bus = open(device.c_str(), O_RDWR);
    if (fd_bus < 0)
    {
        std::cerr << "Failed to open file descriptor!" << std::endl;
        exit(1);
    }
    if (ioctl(fd_bus, I2C_SLAVE, i2c_address) < 0) 
    {
        std::cerr << "I2c communication failed" << std::endl;
        exit(1);
    }

    std::cerr << "I2c communication started. ." << std::endl;
}

Mpu6050Hal::~Mpu6050Hal() { close(fd_bus); }

/*==================================================================================================
*                                       PUBLIC FUNCTIONS
==================================================================================================*/

/*================================================================================================*/
/**
* @brief        Execute I2C read.
* @details      Execute I2C read sequence.
*
* @param[in]    reg         Register address.
* @param[in]    count       Number of bytes to read.
* @param[out]   aRxBuffer   Array to which data will be stored.
*
* @return       Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050Hal::mpu6050_i2c_hal_read(const std::uint8_t reg, std::uint8_t aRxBuffer[], const std::uint8_t count)
{
    /* Param checking */
    if(count == 0U)
    {
        return MPU6050_ERR;
    }
    else
    {
        const auto i2c_ret = i2c_smbus_read_i2c_block_data(fd_bus, reg, count, aRxBuffer);
        if(i2c_ret != static_cast<int>(count))
        {
            return MPU6050_ERR;
        }
    }

    return MPU6050_OK;
}

/*================================================================================================*/
/**
 * @brief        Execute I2C read.
 * @details      Execute I2C read sequence.
 *
 * @param[in]    reg         Register address.
 * @param[out]   RxBuffer    Reference to which data will be stored.
 *
 * @return       Mpu6050_Error_t     Return code.
 *
 */
Mpu6050Hal::Mpu6050_Error_t Mpu6050Hal::mpu6050_i2c_hal_read(const std::uint8_t reg, std::uint8_t &RxBuffer)
{
    std::uint8_t RxBuffer_ {0};

    const auto i2c_ret = i2c_smbus_read_i2c_block_data(fd_bus, reg, 1U, &RxBuffer_);
    if(i2c_ret < 0)
    {
        return MPU6050_ERR;
    }

    RxBuffer = RxBuffer_;

    return MPU6050_OK;
}

/*================================================================================================*/
/**
* @brief        Execute I2C write.
* @details      Execute I2C write sequence.
*
* @param[in]    reg         Register address.
* @param[in]    count       Number of bytes to read.
* @param[out]   aTxBuffer   Array to the data that will be written.
*
* @return       Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050Hal::mpu6050_i2c_hal_write(const std::uint8_t reg, std::uint8_t aTxBuffer[], const std::uint8_t count)
{
    /* Param checking */
    if(count == 0U)
    {
        return MPU6050_ERR;
    }
    else
    {
        const auto i2c_ret = i2c_smbus_write_i2c_block_data(fd_bus, reg, count, aTxBuffer);
        if(i2c_ret < 0)
        {
            return MPU6050_ERR;
        }
    }

    return MPU6050_OK;
}

/*================================================================================================*/
/**
 * @brief        Execute I2C write.
 * @details      Execute I2C write sequence.
 *
 * @param[in]    reg         Register address.
 * @param[out]   TxBuffer    Data that will be written.
 *
 * @return       Mpu6050_Error_t     Return code.
 *
 */
Mpu6050Hal::Mpu6050_Error_t Mpu6050Hal::mpu6050_i2c_hal_write(const std::uint8_t reg, std::uint8_t TxBuffer)
{
    std::uint8_t TxBuffer_ = TxBuffer;

    const auto i2c_ret = i2c_smbus_write_i2c_block_data(fd_bus, reg, 1U, &TxBuffer_);
    if(i2c_ret < 0)
    {
        return MPU6050_ERR;
    }

    TxBuffer = TxBuffer_;

    return MPU6050_OK;
}

/*================================================================================================*/
/**
* @brief        Execute ms delay.
* @details      Execute ms delay for hal usage.
*
* @param[in]    ms      Time in milliseconds.
*
* @return       void
*
*/
void Mpu6050Hal::mpu6050_i2c_hal_ms_delay(std::uint32_t ms) {

    /* User implementation here */
    
    usleep(ms * 1000);
    
}
