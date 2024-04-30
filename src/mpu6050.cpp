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
#include "ros2_mpu6050/mpu6050.h"

#include <iostream>

Mpu6050::Mpu6050(const std::string &device, int i2c_address)
{
    std::uint8_t dev_id {0};
    mpu6050_i2c = std::make_unique<Mpu6050Hal>(device, i2c_address);

    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;

    /* Startup time */
    mpu6050_i2c->mpu6050_i2c_hal_ms_delay(50);

    /* Perform device reset */
    ret |= Mpu6050_Reset();

    mpu6050_i2c->mpu6050_i2c_hal_ms_delay(50);

    /* Set device to normal mode */
    ret |= Mpu6050_SetPowerMode(PWR_MODE_NORMAL);

    ret |= Mpu6050_GetDevideId(dev_id);

    if(Mpu6050Hal::MPU6050_OK == ret && dev_id == WHO_AM_I_VAL)
    {
        std::cout << "MPU6050 initialization successful" << std::endl;
    }
    else
    {
        std::cerr << "MPU6050 initialization failed!" << std::endl;
    }

    mpu6050_i2c->mpu6050_i2c_hal_ms_delay(50);
}

Mpu6050::~Mpu6050() = default;

/*==================================================================================================
*                                       PRIVATE MEMBER FUNCTIONS
==================================================================================================*/

/* Read accelerometer's raw data */
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_GetAccelRawData(Mpu6050_AccelRawData_t &AccelRawData)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_READ_ACCEL_DATA;
    std::uint8_t data[6U] = {0U};

    ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, data, 6U);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        AccelRawData.Accel_X_Raw = static_cast<int16_t>((data[0U] << MSB_8BIT_SHIFT) | data[1U]);
        AccelRawData.Accel_Y_Raw = static_cast<int16_t>((data[2U] << MSB_8BIT_SHIFT) | data[3U]);
        AccelRawData.Accel_Z_Raw = static_cast<int16_t>((data[4U] << MSB_8BIT_SHIFT) | data[5U]);
    }

    return ret;
}

/* Read gyroscope's raw data */
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_GetGyroRawData(Mpu6050_GyroRawData_t &GyroRawData)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_READ_GYRO_DATA;
    std::uint8_t data[6U] = {0U};

    ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, data, 6U);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        GyroRawData.Gyro_X_Raw = static_cast<int16_t>((data[0U] << MSB_8BIT_SHIFT) | data[1U]);
        GyroRawData.Gyro_Y_Raw = static_cast<int16_t>((data[2U] << MSB_8BIT_SHIFT) | data[3U]);
        GyroRawData.Gyro_Z_Raw = static_cast<int16_t>((data[4U] << MSB_8BIT_SHIFT) | data[5U]);
    }

    return ret;
}

/* Read configuration from REG_PWR_MGMT_1 register */
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_ReadPwrMgmt1(std::uint8_t &cfg)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_PWR_MGMT_1;
    std::uint8_t data = 0U;

    ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, data);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        cfg = data;
    }

    return ret;
}

/* Read configuration from REG_PWR_MGMT_2 register */
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_ReadPwrMgmt2(std::uint8_t &cfg)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_PWR_MGMT_2;
    std::uint8_t data = 0U;

    ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, data);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        cfg = data;
    }

    return ret;
}

/*==================================================================================================
*                                       PUBLIC MEMBER FUNCTIONS
==================================================================================================*/

/*================================================================================================*/
/**
* @brief        Set power mode of the device.
* @details      Set power mode of the device (sleep/cycle/normal).
*
* @param[in]    ePwrMode    Power mode.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_SetPowerMode(Mpu6050_PwrMode_t ePwrMode)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_PWR_MGMT_1;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    /* Parameter check */
    if(ePwrMode > PWR_MODE_CYCLE)
    {
        ret |= Mpu6050Hal::MPU6050_ERR;
    }
    else
    {
        ret |= Mpu6050_ReadPwrMgmt1(cfg);

        if (Mpu6050Hal::MPU6050_OK == ret)
        {
            data = cfg;

            switch(ePwrMode)
            {
                case PWR_MODE_NORMAL:
                    data &= (~PWR_MODE_SLEEP_MASK & ~PWR_MODE_CYCLE_MASK);
                    break;
                case PWR_MODE_SLEEP:
                    data |= PWR_MODE_SLEEP_MASK;
                    break;
                case PWR_MODE_CYCLE:
                    data &= ~(PWR_MODE_SLEEP_MASK);
                    data |= PWR_MODE_CYCLE_MASK;
                    break;
                default:
                    /* Invalid parameter */
                    return Mpu6050Hal::MPU6050_ERR;
            }

            ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);;
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Reset device.
* @details      Perform device reset.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_Reset()
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_PWR_MGMT_1;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    ret |= Mpu6050_ReadPwrMgmt1(cfg);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        data = cfg | DEV_RESET_MASK;

        ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Clock select.
* @details      Select clock source.
*
* @param[in]    eClkSrc     PClock source.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_ClockSelect(Mpu6050_ClkSrc_t eClkSrc)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_PWR_MGMT_1;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    /* Parameter check */
    if((eClkSrc > CLK_SRC_STOP) || (eClkSrc == CLK_SRC_RESERVE))
    {
        ret |= Mpu6050Hal::MPU6050_ERR;
    }
    else
    {
        ret |= Mpu6050_ReadPwrMgmt1(cfg);

        if (Mpu6050Hal::MPU6050_OK == ret)
        {
            data = cfg & ~CLKSEL_MASK;
            data |= static_cast<std::uint8_t>(eClkSrc);

            ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);;
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Full scale range setting.
* @details      Configure the accelerometer's full scale range.
*
* @param[in]    eAfsSel     Accelerometer full scale range value.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_AccelFsSel(Mpu6050_AfsSel_t eAfsSel)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_ACCEL_CONFIG;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    /* Parameter check */
    if(eAfsSel > AFSR_16)
    {
        ret |= Mpu6050Hal::MPU6050_ERR;
    }
    else
    {
        ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, cfg);

        if (Mpu6050Hal::MPU6050_OK == ret)
        {
            data = cfg & ~ACCEL_FS_SEL_MASK;
            data |= (static_cast<std::uint8_t>(eAfsSel) << ACCEL_FS_SEL_SHIFT);

            ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);

            /* Confirm acceleration FsSel value and store it in the variable */
            cfg = 0U;
            ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, cfg);


            if (Mpu6050Hal::MPU6050_OK == ret)
            {
                u8AfsSelCurrentVal = (cfg & ACCEL_FS_SEL_MASK) >> ACCEL_FS_SEL_SHIFT;
            }
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Full scale range setting.
* @details      Configure the gyroscope's full scale range.
*
* @param[in]    eFsSel      Gyroscope full scale range value.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_GyroFsSel(Mpu6050_FsSel_t eFsSel)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_GYRO_CONFIG;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    /* Parameter check */
    if(eFsSel > FSR_2000)
    {
        ret |= Mpu6050Hal::MPU6050_ERR;
    }
    else
    {
        ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, cfg);

        if (Mpu6050Hal::MPU6050_OK == ret)
        {
            data = cfg & ~GYRO_FS_SEL_MASK;
            data |= (static_cast<std::uint8_t>(eFsSel) << GYRO_FS_SEL_SHIFT);

            ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);;

            /* Confirm gyroscope FsSel value and store it in the variable */
            cfg = 0U;
            ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, cfg);

            if (Mpu6050Hal::MPU6050_OK == ret)
            {
                u8FsSelCurrentVal = (cfg & GYRO_FS_SEL_MASK) >> GYRO_FS_SEL_SHIFT;
            }
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Configures the DLPF setting.
* @details      Configures the digital low pass filter setting.
*
* @param[in]    eDlpfCfg    Digital low pass filter setting value.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_DlpfConfig(Mpu6050_DlpfCfg_t eDlpfCfg)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_CONFIG;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    /* Parameter check */
    if(eDlpfCfg > DLPF_CFG_5_5)
    {
        ret |= Mpu6050Hal::MPU6050_ERR;
    }
    else
    {
        ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, cfg);

        if (Mpu6050Hal::MPU6050_OK == ret)
        {
            data = cfg & ~DLPF_CFG_MASK;
            data |= static_cast<std::uint8_t>(eDlpfCfg);

            ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);;
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Low power wake control.
* @details      Specifies the frequency of wake-ups during Accelerometer Only Low PowerMode.
*
* @param[in]    eLpWakeCtrl Wake control frequency (Hz).
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_LpWakeCtrl(Mpu6050_LpWakeCtrl_t eLpWakeCtrl)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_PWR_MGMT_1;
    std::uint8_t cfg = 0U;
    std::uint8_t data = 0U;

    /* Parameter check */
    if(eLpWakeCtrl > WAKE_CTRL_40HZ)
    {
        ret |= Mpu6050Hal::MPU6050_ERR;
    }
    else
    {
        ret |= Mpu6050_ReadPwrMgmt2(cfg);

        if (Mpu6050Hal::MPU6050_OK == ret)
        {
            data = cfg & ~LP_WAKE_CTRL_MASK;
            data |= (static_cast<std::uint8_t>(eLpWakeCtrl) << LP_WAKE_CTRL_SHIFT);

            ret |= mpu6050_i2c->mpu6050_i2c_hal_write(reg, data);;
        }
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Get accelerometer data.
* @details      Get 3 axis accelerometer data.
*
* @param[in]    AccelData   Pointer where the data will be stored.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_GetAccelData(Mpu6050_AccelData_t &AccelData)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    Mpu6050_AccelRawData_t AccelRawData;

    ret |= Mpu6050_GetAccelRawData(AccelRawData);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        AccelData.Accel_X = static_cast<double>((static_cast<double>(AccelRawData.Accel_X_Raw) * ACCEL_GRAVITY) / aAfsSelVal[u8AfsSelCurrentVal]);
        AccelData.Accel_Y = static_cast<double>((static_cast<double>(AccelRawData.Accel_Y_Raw) * ACCEL_GRAVITY) / aAfsSelVal[u8AfsSelCurrentVal]);
        AccelData.Accel_Z = static_cast<double>((static_cast<double>(AccelRawData.Accel_Z_Raw) * ACCEL_GRAVITY) / aAfsSelVal[u8AfsSelCurrentVal]);
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Get gyroscope data.
* @details      Get 3 axis gyroscope data.
*
* @param[in]    GyroData    Pointer where the data will be stored.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_GetGyroData(Mpu6050_GyroData_t &GyroData)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    Mpu6050_GyroRawData_t GyroRawData;

    ret |= Mpu6050_GetGyroRawData(GyroRawData);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        GyroData.Gyro_X = static_cast<double>(static_cast<double>(GyroRawData.Gyro_X_Raw) / aFsSelVal[u8FsSelCurrentVal]);
        GyroData.Gyro_Y = static_cast<double>(static_cast<double>(GyroRawData.Gyro_Y_Raw) / aFsSelVal[u8FsSelCurrentVal]);
        GyroData.Gyro_Z = static_cast<double>(static_cast<double>(GyroRawData.Gyro_Z_Raw) / aFsSelVal[u8FsSelCurrentVal]);
    }

    return ret;
}

/*================================================================================================*/
/**
* @brief        Read device ID.
* @details      Read 8bit device id.
*
* @param[in]    pId         Pointer to which the data will be stored.
*
* @return       Mpu6050Hal::Mpu6050_Error_t     Return code.
*
*/
Mpu6050Hal::Mpu6050_Error_t Mpu6050::Mpu6050_GetDevideId(std::uint8_t &pId)
{
    Mpu6050Hal::Mpu6050_Error_t ret = Mpu6050Hal::MPU6050_OK;
    const std::uint8_t reg = REG_WHO_AM_I;
    std::uint8_t data = 0U;

    ret |= mpu6050_i2c->mpu6050_i2c_hal_read(reg, data);

    if (Mpu6050Hal::MPU6050_OK == ret)
    {
        pId = (data & WHO_AM_I_MASK);
    }

    return ret;
}
