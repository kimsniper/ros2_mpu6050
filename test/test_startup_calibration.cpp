#include "ros2_mpu6050/startup_calibration.h"

#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

namespace
{
using ros2_mpu6050::calibration::StartupCalibrationResult;

StartupCalibrationResult valid_result()
{
    StartupCalibrationResult result {};

    result.acceleration[2U].mean = 9.81;
    result.acceleration_norm.mean = 9.81;
    result.acceleration_norm.standard_deviation = 0.05;
    result.gravity_magnitude_error = 0.0;

    result.gyro[0U] = {1.9, 0.3};
    result.gyro[1U] = {1.0, 0.2};
    result.gyro[2U] = {-0.3, 0.1};

    return result;
}
}  // namespace

TEST(StartupCalibration, AcceptsStationaryStatistics)
{
    EXPECT_NO_THROW(ros2_mpu6050::calibration::validate_result(valid_result()));
}

TEST(StartupCalibration, RejectsAccelerationMovement)
{
    auto result = valid_result();
    result.acceleration_norm.standard_deviation = 0.151;

    EXPECT_THROW(ros2_mpu6050::calibration::validate_result(result), std::runtime_error);
}

TEST(StartupCalibration, RejectsGyroscopeMovement)
{
    auto result = valid_result();
    result.gyro[1U].standard_deviation = 0.501;

    EXPECT_THROW(ros2_mpu6050::calibration::validate_result(result), std::runtime_error);
}

TEST(StartupCalibration, RejectsIncorrectGravityMagnitude)
{
    auto result = valid_result();
    result.gravity_magnitude_error = 1.001;

    EXPECT_THROW(ros2_mpu6050::calibration::validate_result(result), std::runtime_error);
}

TEST(StartupCalibration, RejectsNonFiniteGyroBias)
{
    auto result = valid_result();
    result.gyro[0U].mean = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW(ros2_mpu6050::calibration::validate_result(result), std::runtime_error);
}

TEST(StartupCalibration, StatisticsRejectZeroSamples)
{
    EXPECT_THROW(ros2_mpu6050::calibration::calculate_statistics(0.0, 0.0, 0U), std::invalid_argument);
}

TEST(StartupCalibration, StatisticsCalculateMeanAndDeviation)
{
    const auto statistics = ros2_mpu6050::calibration::calculate_statistics(6.0, 14.0, 3U);

    EXPECT_DOUBLE_EQ(statistics.mean, 2.0);
    EXPECT_NEAR(statistics.standard_deviation, 0.816496580927726, 1.0e-12);
}
