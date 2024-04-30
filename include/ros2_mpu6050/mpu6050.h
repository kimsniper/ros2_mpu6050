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

#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include "ros2_mpu6050/mpu6050_hal.h"

#include <string>
#include <memory>
#include <cstdint>

class Mpu6050 {
public:
    explicit Mpu6050(const std::string &device = "/dev/i2c-1", int i2c_address = I2C_ADDRESS_MPU6050_AD0_L);

    ~Mpu6050();

    typedef enum{
        DLPF_CFG_260_256 = 0x00U,
        DLPF_CFG_184_188 = 0x01U,
        DLPF_CFG_94_98 = 0x02U,
        DLPF_CFG_44_42 = 0x03U,
        DLPF_CFG_21_20 = 0x04U,
        DLPF_CFG_10_10 = 0x05U,
        DLPF_CFG_5_5 = 0x06U
    } Mpu6050_DlpfCfg_t;

    typedef enum{
        FSR_250 = 0x00U,
        FSR_500 = 0x01U,
        FSR_1000 = 0x02U,
        FSR_2000 = 0x03U
    } Mpu6050_FsSel_t;

    typedef enum{
        AFSR_2 = 0x00U,
        AFSR_4 = 0x01U,
        AFSR_8 = 0x02U,
        AFSR_16 = 0x03U
    } Mpu6050_AfsSel_t;

    typedef enum{
        PWR_MODE_NORMAL = 0x00U,
        PWR_MODE_SLEEP = 0x01U,
        PWR_MODE_CYCLE = 0x02U,
    } Mpu6050_PwrMode_t;

    typedef enum{
        CLK_SRC_8MHZ = 0x00U,
        CLK_SRC_PLL_X_GYRO_REF = 0x01U,
        CLK_SRC_PLL_Y_GYRO_REF = 0x02U,
        CLK_SRC_PLL_Z_GYRO_REF = 0x03U,
        CLK_SRC_PLL_32MHZ = 0x04U,
        CLK_SRC_PLL_19MHZ = 0x05U,
        CLK_SRC_RESERVE = 0x06U,
        CLK_SRC_STOP = 0x07U
    } Mpu6050_ClkSrc_t;

    typedef enum{
        WAKE_CTRL_1P25HZ = 0x00U,
        WAKE_CTRL_5HZ = 0x01U,
        WAKE_CTRL_20HZ = 0x02U,
        WAKE_CTRL_40HZ = 0x03U
    } Mpu6050_LpWakeCtrl_t;

    /**
     * @brief Accelerometer raw data
     */
    typedef struct{
        int16_t Accel_X_Raw;     /** @brief Accel X raw data */
        int16_t Accel_Y_Raw;     /** @brief Accel Y raw data */
        int16_t Accel_Z_Raw;     /** @brief Accel Z raw data */
    } Mpu6050_AccelRawData_t;

    /**
     * @brief Accelerometer data
     */
    typedef struct{
        double Accel_X;     /** @brief Accel X data */
        double Accel_Y;     /** @brief Accel Y data */
        double Accel_Z;     /** @brief Accel Z data */
    } Mpu6050_AccelData_t;

    /**
     * @brief Gyroscope raw data
     */
    typedef struct{
        int16_t Gyro_X_Raw;     /** @brief Gyro X raw data */
        int16_t Gyro_Y_Raw;     /** @brief Gyro Y raw data */
        int16_t Gyro_Z_Raw;     /** @brief Gyro Z raw data */
    } Mpu6050_GyroRawData_t;

    /**
     * @brief Gyroscope data
     */
    typedef struct{
        double Gyro_X;     /** @brief Gyro X data */
        double Gyro_Y;     /** @brief Gyro Y data */
        double Gyro_Z;     /** @brief Gyro Z data */
    } Mpu6050_GyroData_t;

    /*==================================================================================================
    *                                       FUNCTION Prototypes
    ==================================================================================================*/

    /**
    * @brief        Set power mode of the device.
    * @details      Set power mode of the device (sleep/cycle/normal).
    *
    * @param[in]    ePwrMode    Power mode.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_SetPowerMode(Mpu6050_PwrMode_t ePwrMode);

    /**
    * @brief        Reset device.
    * @details      Perform device reset.
    *
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_Reset();

    /**
    * @brief        Clock select.
    * @details      Select clock source.
    *
    * @param[in]    eClkSrc     PClock source.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_ClockSelect(Mpu6050_ClkSrc_t eClkSrc);

    /**
    * @brief        Full scale range setting.
    * @details      Configure the accelerometer's full scale range.
    *
    * @param[in]    eAfsSel     Accelerometer full scale range value.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_AccelFsSel(Mpu6050_AfsSel_t eAfsSel);

    /**
    * @brief        Full scale range setting.
    * @details      Configure the gyroscope's full scale range.
    *
    * @param[in]    eFsSel      Gyroscope full scale range value.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_GyroFsSel(Mpu6050_FsSel_t eFsSel);

    /**
    * @brief        Configures the DLPF setting.
    * @details      Configures the digital low pass filter setting.
    *
    * @param[in]    eDlpfCfg    Digital low pass filter setting value.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_DlpfConfig(Mpu6050_DlpfCfg_t eDlpfCfg);

    /**
    * @brief        Low power wake control.
    * @details      Specifies the frequency of wake-ups during Accelerometer Only Low PowerMode.
    *
    * @param[in]    eLpWakeCtrl Wake control frequency (Hz).
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_LpWakeCtrl(Mpu6050_LpWakeCtrl_t eLpWakeCtrl);

    /**
    * @brief        Get accelerometer data.
    * @details      Get 3 axis accelerometer data.
    *
    * @param[in]    AccelData   Pointer where the data will be stored.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_GetAccelData(Mpu6050_AccelData_t &AccelData);
    /**
    * @brief        Get gyroscope data.
    * @details      Get 3 axis gyroscope data.
    *
    * @param[in]    GyroData    Pointer where the data will be stored.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_GetGyroData(Mpu6050_GyroData_t &GyroData);

    /**
    * @brief        Read device ID.
    * @details      Read 8bit device id.
    *
    * @param[in]    pId         Pointer to which the data will be stored.
    *
    * @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
    *
    */
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_GetDevideId(std::uint8_t &pId);

private:
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_GetAccelRawData(Mpu6050_AccelRawData_t &AccelRawData);
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_GetGyroRawData(Mpu6050_GyroRawData_t &GyroRawData);
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_ReadPwrMgmt1(std::uint8_t &cfg);
    Mpu6050Hal::Mpu6050_Error_t Mpu6050_ReadPwrMgmt2(std::uint8_t &cfg);

    std::unique_ptr<Mpu6050Hal> mpu6050_i2c;

    std::uint8_t u8FsSelCurrentVal = static_cast<std::uint8_t>(FSR_250);
    std::uint8_t u8AfsSelCurrentVal = static_cast<std::uint8_t>(AFSR_2);
    const std::array<double, 4> aFsSelVal {131, 65.5, 32.8, 16.4};
    const std::array<double, 4> aAfsSelVal {16384, 8192, 4096, 2048};

    /**
     * @brief MPU6050 I2C slave address when pin AD0 is low
     */
    static constexpr std::uint8_t I2C_ADDRESS_MPU6050_AD0_L       = 0x68;

    /**
     * @brief MPU6050 I2C slave address when pin AD0 is high
     */
    static constexpr std::uint8_t I2C_ADDRESS_MPU6050_AD0_H       = 0x69;

    /**
     * @brief MPU6050 command code registers
     */
    static constexpr std::uint8_t REG_CONFIG                      = 0x1A;
    static constexpr std::uint8_t REG_GYRO_CONFIG                 = 0x1B;
    static constexpr std::uint8_t REG_ACCEL_CONFIG                = 0x1C;
    static constexpr std::uint8_t REG_MOT_THR                     = 0x1F;
    static constexpr std::uint8_t REG_FIFO_EN                     = 0x23;
    static constexpr std::uint8_t REG_READ_ACCEL_DATA             = 0x3B;
    static constexpr std::uint8_t REG_READ_TEMP_DATA              = 0x41;
    static constexpr std::uint8_t REG_READ_GYRO_DATA              = 0x43;
    static constexpr std::uint8_t REG_PWR_MGMT_1                  = 0x6B;
    static constexpr std::uint8_t REG_PWR_MGMT_2                  = 0x6C;
    static constexpr std::uint8_t REG_WHO_AM_I                    = 0x75;

    /**
     * @brief Mask and shift variables
     */
    static constexpr std::uint8_t FIFO_EN_MASK                    = 0x40;
    static constexpr std::uint8_t FIFO_RESET_MASK                 = 0x04;
    static constexpr std::uint8_t FIFO_TEMP_EN_MASK               = 0x80;
    static constexpr std::uint8_t FIFO_XG_EN_MASK                 = 0x40;
    static constexpr std::uint8_t FIFO_YG_EN_MASK                 = 0x20;
    static constexpr std::uint8_t FIFO_ZG_EN_MASK                 = 0x10;
    static constexpr std::uint8_t FIFO_ACCEL_EN_MASK              = 0x08;
    static constexpr std::uint8_t FIFO_SLV2_EN_MASK               = 0x04;
    static constexpr std::uint8_t FIFO_SLV1_EN_MASK               = 0x02;
    static constexpr std::uint8_t FIFO_SLV0_EN_MASK               = 0x01;

    static constexpr std::uint8_t GYRO_FS_SEL_MASK                = 0x18;
    static constexpr std::uint8_t GYRO_FS_SEL_SHIFT               = 0x03;

    static constexpr std::uint8_t ACCEL_FS_SEL_MASK               = 0x18;
    static constexpr std::uint8_t ACCEL_FS_SEL_SHIFT              = 0x03;

    static constexpr std::uint8_t DLPF_CFG_MASK                   = 0x07;

    /* PWR_MGMT_1 related variables */
    static constexpr std::uint8_t DEV_RESET_MASK                  = 0x80;
    static constexpr std::uint8_t PWR_MODE_SLEEP_MASK             = 0x40;
    static constexpr std::uint8_t PWR_MODE_CYCLE_MASK             = 0x20;
    static constexpr std::uint8_t TEMP_DIS_MASK                   = 0x08;
    static constexpr std::uint8_t CLKSEL_MASK                     = 0x07;

    /* PWR_MGMT_2 related macros */
    static constexpr std::uint8_t LP_WAKE_CTRL_MASK               = 0xC0;
    static constexpr std::uint8_t LP_WAKE_CTRL_SHIFT              = 0x06;
    static constexpr std::uint8_t STBY_XA                         = 0x20;
    static constexpr std::uint8_t STBY_YA                         = 0x10;
    static constexpr std::uint8_t STBY_ZA                         = 0x08;
    static constexpr std::uint8_t STBY_XG                         = 0x04;
    static constexpr std::uint8_t STBY_YG                         = 0x02;
    static constexpr std::uint8_t STBY_ZG                         = 0x01;

    static constexpr std::uint8_t WHO_AM_I_MASK                   = 0x7E;

    static constexpr std::uint8_t MSB_8BIT_SHIFT                  = 0x08;

    /**
     * @brief Other macros
     */
    static constexpr std::uint8_t WHO_AM_I_VAL                    = 0x68;
    static constexpr std::uint8_t TRUE                            = 0x01;
    static constexpr std::uint8_t FALSE                           = 0x00;
    static constexpr double ACCEL_GRAVITY                         = 9.81;
};

#endif  // MPU6050SENSOR_H
