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

#ifndef ROS2_MPU6050__STARTUP_CALIBRATION_H_
#define ROS2_MPU6050__STARTUP_CALIBRATION_H_

#include "ros2_mpu6050/mpu6050.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <thread>

namespace ros2_mpu6050
{
namespace calibration
{

constexpr std::size_t kDefaultWarmupSamples = 100U;
constexpr std::size_t kDefaultCalibrationSamples = 500U;
constexpr double kGravity = 9.81;
constexpr double kMaximumGravityMagnitudeError = 1.0;
constexpr double kMaximumAccelerationNormStdDev = 0.15;
constexpr double kMaximumGyroAxisStdDev = 0.5;
constexpr auto kDefaultSamplePeriod = std::chrono::milliseconds(10);

struct AxisStatistics
{
    double mean {0.0};
    double standard_deviation {0.0};
};

struct StartupCalibrationResult
{
    std::array<AxisStatistics, 3U> acceleration {};
    std::array<AxisStatistics, 3U> gyro {};
    AxisStatistics acceleration_norm {};
    double gravity_magnitude_error {0.0};
};

inline AxisStatistics calculate_statistics(double sum, double sum_of_squares, std::size_t sample_count)
{
    if (sample_count == 0U)
    {
        throw std::invalid_argument("Calibration sample count must be greater than zero");
    }

    const double count = static_cast<double>(sample_count);
    const double mean = sum / count;
    const double mean_square = sum_of_squares / count;
    const double variance = std::max(0.0, mean_square - (mean * mean));

    return {mean, std::sqrt(variance)};
}

inline void validate_result(const StartupCalibrationResult& result)
{
    if (!std::isfinite(result.acceleration_norm.mean) ||
        !std::isfinite(result.acceleration_norm.standard_deviation) ||
        !std::isfinite(result.gravity_magnitude_error))
    {
        throw std::runtime_error("Calibration rejected: non-finite acceleration statistics");
    }

    if (result.gravity_magnitude_error >
        kMaximumGravityMagnitudeError)
    {
        throw std::runtime_error("Calibration rejected: acceleration magnitude is not consistent with stationary gravity");
    }

    if (result.acceleration_norm.standard_deviation > kMaximumAccelerationNormStdDev)
    {
        throw std::runtime_error("Calibration rejected: accelerometer data indicates movement");
    }

    constexpr const char* axis_names[] = {"x", "y", "z"};

    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (!std::isfinite(result.gyro[axis].mean) || !std::isfinite(result.gyro[axis].standard_deviation))
        {
            throw std::runtime_error(std::string("Calibration rejected: non-finite gyro ") + axis_names[axis] + " statistics");
        }

        if (result.gyro[axis].standard_deviation > kMaximumGyroAxisStdDev)
        {
            throw std::runtime_error(std::string("Calibration rejected: gyro ") + axis_names[axis] + " indicates movement");
        }
    }
}

inline StartupCalibrationResult run_startup_calibration(
    Mpu6050& sensor,
    std::size_t warmup_samples = kDefaultWarmupSamples,
    std::size_t calibration_samples = kDefaultCalibrationSamples,
    std::chrono::milliseconds sample_period = kDefaultSamplePeriod)
{
    if (calibration_samples == 0U)
    {
        throw std::invalid_argument("Calibration sample count must be greater than zero");
    }

    Mpu6050::Mpu6050_AccelData_t acceleration_data {};
    Mpu6050::Mpu6050_GyroData_t gyro_data {};

    for (std::size_t sample = 0U; sample < warmup_samples; ++sample)
    {
        if (Mpu6050Hal::MPU6050_OK != sensor.Mpu6050_GetImuData(acceleration_data, gyro_data))
        {
            throw std::runtime_error("Failed to read a warm-up MPU6050 sample");
        }

        std::this_thread::sleep_for(sample_period);
    }

    std::array<double, 3U> acceleration_sum {};
    std::array<double, 3U> acceleration_square_sum {};
    std::array<double, 3U> gyro_sum {};
    std::array<double, 3U> gyro_square_sum {};
    double acceleration_norm_sum {0.0};
    double acceleration_norm_square_sum {0.0};

    for (std::size_t sample = 0U; sample < calibration_samples; ++sample)
    {
        if (Mpu6050Hal::MPU6050_OK != sensor.Mpu6050_GetImuData(acceleration_data, gyro_data))
        {
            throw std::runtime_error("Failed to read an MPU6050 calibration sample");
        }

        const std::array<double, 3U> acceleration {acceleration_data.Accel_X, acceleration_data.Accel_Y, acceleration_data.Accel_Z};
        const std::array<double, 3U> gyro {gyro_data.Gyro_X, gyro_data.Gyro_Y, gyro_data.Gyro_Z};

        const double acceleration_norm = std::sqrt(acceleration[0U] * acceleration[0U] + acceleration[1U] * acceleration[1U] + acceleration[2U] * acceleration[2U]);

        if (!std::isfinite(acceleration_norm))
        {
            throw std::runtime_error("Non-finite acceleration received during calibration");
        }

        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            if (!std::isfinite(acceleration[axis]) || !std::isfinite(gyro[axis]))
            {
                throw std::runtime_error("Non-finite MPU6050 data received during calibration");
            }

            acceleration_sum[axis] += acceleration[axis];
            acceleration_square_sum[axis] += acceleration[axis] * acceleration[axis];
            gyro_sum[axis] += gyro[axis];
            gyro_square_sum[axis] += gyro[axis] * gyro[axis];
        }

        acceleration_norm_sum += acceleration_norm;
        acceleration_norm_square_sum += acceleration_norm * acceleration_norm;

        std::this_thread::sleep_for(sample_period);
    }

    StartupCalibrationResult result {};

    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        result.acceleration[axis] = calculate_statistics(acceleration_sum[axis], acceleration_square_sum[axis], calibration_samples);
        result.gyro[axis] = calculate_statistics(gyro_sum[axis], gyro_square_sum[axis], calibration_samples);
    }

    result.acceleration_norm = calculate_statistics(acceleration_norm_sum, acceleration_norm_square_sum, calibration_samples);

    const double measured_gravity_norm = std::sqrt(result.acceleration[0U].mean * result.acceleration[0U].mean +
                                                result.acceleration[1U].mean * result.acceleration[1U].mean +
                                                result.acceleration[2U].mean * result.acceleration[2U].mean);

    if (!std::isfinite(measured_gravity_norm) || measured_gravity_norm < 1.0)
    {
        throw std::runtime_error("Calibration rejected: invalid gravity vector");
    }

    result.gravity_magnitude_error = std::abs(measured_gravity_norm - kGravity);

    validate_result(result);
    return result;
}

}  // namespace calibration
}  // namespace ros2_mpu6050

#endif  // ROS2_MPU6050__STARTUP_CALIBRATION_H_
