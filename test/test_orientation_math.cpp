#include "ros2_mpu6050/orientation_math.h"

#include <cmath>

#include "gtest/gtest.h"

namespace
{
constexpr double kTolerance = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

double quaternion_norm(
    const ros2_mpu6050::orientation::Quaternion& quaternion)
{
    return std::sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w);
}
}  // namespace

TEST(OrientationMath, LevelAccelerationProducesZeroTilt)
{
    const auto tilt = ros2_mpu6050::orientation::calculate_accelerometer_tilt(0.0, 0.0, 9.81);

    EXPECT_NEAR(tilt.roll, 0.0, kTolerance);
    EXPECT_NEAR(tilt.pitch, 0.0, kTolerance);
}

TEST(OrientationMath, PositiveYGravityProducesPositiveNinetyDegreeRoll)
{
    const auto tilt = ros2_mpu6050::orientation::calculate_accelerometer_tilt(0.0, 9.81, 0.0);

    EXPECT_NEAR(tilt.roll, kPi / 2.0, kTolerance);
    EXPECT_NEAR(tilt.pitch, 0.0, kTolerance);
}

TEST(OrientationMath, PositiveXGravityProducesNegativeNinetyDegreePitch)
{
    const auto tilt = ros2_mpu6050::orientation::calculate_accelerometer_tilt(9.81, 0.0, 0.0);

    EXPECT_NEAR(tilt.pitch, -kPi / 2.0, kTolerance);
}

TEST(OrientationMath, InitializationUsesTiltAndZerosYaw)
{
    ros2_mpu6050::orientation::EulerAngles orientation {1.0, 2.0, 3.0};
    const ros2_mpu6050::orientation::EulerAngles tilt {0.25, -0.5, 8.0};

    ros2_mpu6050::orientation::initialize_orientation(orientation, tilt);

    EXPECT_NEAR(orientation.roll, 0.25, kTolerance);
    EXPECT_NEAR(orientation.pitch, -0.5, kTolerance);
    EXPECT_NEAR(orientation.yaw, 0.0, kTolerance);
}

TEST(OrientationMath, StationaryComplementaryUpdatePreservesLevelPose)
{
    ros2_mpu6050::orientation::EulerAngles orientation {};
    const ros2_mpu6050::orientation::EulerAngles tilt {};

    ros2_mpu6050::orientation::update_complementary_filter(
        orientation,
        tilt,
        0.0,
        0.0,
        0.0,
        0.01,
        0.98);

    EXPECT_NEAR(orientation.roll, 0.0, kTolerance);
    EXPECT_NEAR(orientation.pitch, 0.0, kTolerance);
    EXPECT_NEAR(orientation.yaw, 0.0, kTolerance);
}

TEST(OrientationMath, GyroscopeZIntegratesRelativeYaw)
{
    ros2_mpu6050::orientation::EulerAngles orientation {};
    const ros2_mpu6050::orientation::EulerAngles tilt {};

    ros2_mpu6050::orientation::update_complementary_filter(orientation,tilt, 0.0, 0.0, kPi / 2.0, 1.0, 0.98);

    EXPECT_NEAR(orientation.yaw, kPi / 2.0, kTolerance);
}

TEST(OrientationMath, QuaternionIsNormalizedForArbitraryOrientation)
{
    const ros2_mpu6050::orientation::EulerAngles orientation {0.31, -0.72, 1.48};

    const auto quaternion = ros2_mpu6050::orientation::euler_to_quaternion(orientation);

    EXPECT_NEAR(quaternion_norm(quaternion), 1.0, kTolerance);
}

TEST(OrientationMath, NinetyDegreeYawProducesExpectedQuaternion)
{
    const ros2_mpu6050::orientation::EulerAngles orientation {0.0, 0.0, kPi / 2.0};

    const auto quaternion = ros2_mpu6050::orientation::euler_to_quaternion(orientation);

    const double expected = std::sqrt(0.5);

    EXPECT_NEAR(quaternion.x, 0.0, kTolerance);
    EXPECT_NEAR(quaternion.y, 0.0, kTolerance);
    EXPECT_NEAR(quaternion.z, expected, kTolerance);
    EXPECT_NEAR(quaternion.w, expected, kTolerance);
    EXPECT_NEAR(quaternion_norm(quaternion), 1.0, kTolerance);
}
