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

#ifndef ROS2_MPU6050__ORIENTATION_MATH_H_
#define ROS2_MPU6050__ORIENTATION_MATH_H_

#include <cmath>

namespace ros2_mpu6050
{
namespace orientation
{

struct EulerAngles
{
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};
};

struct Quaternion
{
    double x {0.0};
    double y {0.0};
    double z {0.0};
    double w {1.0};
};

inline EulerAngles calculate_accelerometer_tilt(double accel_x, double accel_y, double accel_z)
{
    return {std::atan2(accel_y, accel_z), std::atan2(-accel_x, std::sqrt((accel_y * accel_y) + (accel_z * accel_z))), 0.0};
}

inline void initialize_orientation(EulerAngles& orientation, const EulerAngles& accelerometer_tilt)
{
    orientation.roll = accelerometer_tilt.roll;
    orientation.pitch = accelerometer_tilt.pitch;
    orientation.yaw = 0.0;
}

inline void update_complementary_filter(
    EulerAngles& orientation,
    const EulerAngles& accelerometer_tilt,
    double gyro_x,
    double gyro_y,
    double gyro_z,
    double dt,
    double alpha)
{
    orientation.roll = alpha * (orientation.roll + gyro_x * dt) + (1.0 - alpha) * accelerometer_tilt.roll;

    orientation.pitch = alpha * (orientation.pitch + gyro_y * dt) + (1.0 - alpha) * accelerometer_tilt.pitch;

    orientation.yaw += gyro_z * dt;
}

inline Quaternion euler_to_quaternion(const EulerAngles& orientation)
{
    const double half_roll = orientation.roll * 0.5;
    const double half_pitch = orientation.pitch * 0.5;
    const double half_yaw = orientation.yaw * 0.5;

    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);

    Quaternion quaternion {sr * cp * cy - cr * sp * sy,
                            cr * sp * cy + sr * cp * sy,
                            cr * cp * sy - sr * sp * cy,
                            cr * cp * cy + sr * sp * sy};

    const double norm = std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w);

    if (std::isfinite(norm) && norm > 0.0)
    {
        quaternion.x /= norm;
        quaternion.y /= norm;
        quaternion.z /= norm;
        quaternion.w /= norm;
    }

    return quaternion;
}

}  // namespace orientation
}  // namespace ros2_mpu6050

#endif  // ROS2_MPU6050__ORIENTATION_MATH_H_
