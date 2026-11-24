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
#include "ros2_mpu6050/startup_calibration.h"

#include <iomanip>
#include <iostream>
#include <memory>

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    try
    {
        auto sensor = std::make_unique<Mpu6050>();

        if (Mpu6050Hal::MPU6050_OK != sensor->Mpu6050_GyroFsSel(Mpu6050::FSR_250) ||
            Mpu6050Hal::MPU6050_OK != sensor->Mpu6050_AccelFsSel(Mpu6050::AFSR_2) ||
            Mpu6050Hal::MPU6050_OK != sensor->Mpu6050_DlpfConfig(Mpu6050::DLPF_CFG_44_42) ||
            Mpu6050Hal::MPU6050_OK != sensor->Mpu6050_SampleRateDivider(9U) ||
            Mpu6050Hal::MPU6050_OK != sensor->Mpu6050_ClockSelect(Mpu6050::CLK_SRC_PLL_X_GYRO_REF))
        {
            throw std::runtime_error("Failed to configure MPU6050 for calibration");
        }

        std::cout << "Keep the sensor completely stationary." << std::endl;

        const auto result = ros2_mpu6050::calibration::run_startup_calibration(*sensor);

        std::cout << std::fixed << std::setprecision(9)
                  << "gyro_x_offset: "
                  << result.gyro[0U].mean << '\n'
                  << "gyro_y_offset: "
                  << result.gyro[1U].mean << '\n'
                  << "gyro_z_offset: "
                  << result.gyro[2U].mean << '\n'
                  << "acceleration_norm_mean: "
                  << result.acceleration_norm.mean << '\n'
                  << "acceleration_norm_stddev: "
                  << result.acceleration_norm.standard_deviation << '\n'
                  << "gravity_magnitude_error: "
                  << result.gravity_magnitude_error << std::endl;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Calibration failed: " << error.what() << std::endl;
        return 1;
    }

    return 0;
}