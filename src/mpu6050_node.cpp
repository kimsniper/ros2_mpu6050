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

#include "ros2_mpu6050/mpu6050_node.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node(const std::string& name)
    : Node(name)
    , mpu6050_dev_{std::make_unique<Mpu6050>()}
{
    // Declare parameters
    this->declare_parameter<int>("gyro_fs_sel", 0);
    this->declare_parameter<int>("accel_afs_sel", 0);
    this->declare_parameter<int>("dlpf_cfg", 3);
    this->declare_parameter<int>("sample_rate_divider", 9);
    this->declare_parameter<int>("clock_src", 1);
    this->declare_parameter<double>("complementary_filter_alpha", 0.98);
    this->declare_parameter<std::string>("frame_id", "imu_link");

    const auto gyro_fs_sel = this->get_parameter("gyro_fs_sel").as_int();
    const auto accel_afs_sel = this->get_parameter("accel_afs_sel").as_int();
    const auto dlpf_cfg = this->get_parameter("dlpf_cfg").as_int();
    const auto sample_rate_divider = this->get_parameter("sample_rate_divider").as_int();
    const auto clock_src = this->get_parameter("clock_src").as_int();

    complementary_filter_alpha_ = this->get_parameter("complementary_filter_alpha").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    const auto require_integer_range = [](const std::string& parameter_name,
                                        std::int64_t value,
                                        std::int64_t minimum,
                                        std::int64_t maximum)
    {
        if (value < minimum || value > maximum) {
            throw std::invalid_argument(parameter_name + " must be between " + std::to_string(minimum) + " and " + std::to_string(maximum));
        }
    };

    require_integer_range("gyro_fs_sel", gyro_fs_sel, 0, 3);
    require_integer_range("accel_afs_sel", accel_afs_sel, 0, 3);
    require_integer_range("dlpf_cfg", dlpf_cfg, 0, 6);
    require_integer_range("sample_rate_divider", sample_rate_divider, 0, 255);
    require_integer_range("clock_src", clock_src, 0, 5);

    if (!std::isfinite(complementary_filter_alpha_) || complementary_filter_alpha_ < 0.0 || complementary_filter_alpha_ > 1.0)
    {
        throw std::invalid_argument("complementary_filter_alpha must be between 0.0 and 1.0");
    }

    if (frame_id_.empty()) {
        throw std::invalid_argument("frame_id must not be empty");
    }

    Mpu6050Hal::Mpu6050_Error_t configuration_status = Mpu6050Hal::MPU6050_OK;

    configuration_status |= mpu6050_dev_->Mpu6050_GyroFsSel(static_cast<Mpu6050::Mpu6050_FsSel_t>(gyro_fs_sel));
    configuration_status |= mpu6050_dev_->Mpu6050_AccelFsSel(static_cast<Mpu6050::Mpu6050_AfsSel_t>(accel_afs_sel));
    configuration_status |= mpu6050_dev_->Mpu6050_DlpfConfig(static_cast<Mpu6050::Mpu6050_DlpfCfg_t>(dlpf_cfg));
    configuration_status |= mpu6050_dev_->Mpu6050_SampleRateDivider(static_cast<std::uint8_t>(sample_rate_divider));
    configuration_status |= mpu6050_dev_->Mpu6050_ClockSelect(static_cast<Mpu6050::Mpu6050_ClkSrc_t>(clock_src));

    if (Mpu6050Hal::MPU6050_OK != configuration_status) {
        throw std::runtime_error("Failed to configure MPU6050");
    }

    RCLCPP_INFO(this->get_logger(), "Starting MPU6050 stationary calibration. Keep the robot still.");

    const auto calibration_result = ros2_mpu6050::calibration::run_startup_calibration(*mpu6050_dev_);

    gyro_x_offset_ = calibration_result.gyro[0U].mean;
    gyro_y_offset_ = calibration_result.gyro[1U].mean;
    gyro_z_offset_ = calibration_result.gyro[2U].mean;

    RCLCPP_INFO(this->get_logger(),
                "MPU6050 calibration passed: "
                "gyro bias [deg/s] x=%.9f, y=%.9f, z=%.9f; "
                "acceleration norm stddev=%.9f m/s^2; "
                "gravity magnitude error=%.9f m/s^2",
                gyro_x_offset_,
                gyro_y_offset_,
                gyro_z_offset_,
                calibration_result.acceleration_norm.standard_deviation,
                calibration_result.gravity_magnitude_error);

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/mpu6050", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&Mpu6050Node::ImuPubCallback, this));
}

void Mpu6050Node::ImuPubCallback()
{
    auto message = sensor_msgs::msg::Imu();
    const auto current_time = this->get_clock()->now();
    message.header.stamp = current_time;
    message.header.frame_id = frame_id_;
    message.linear_acceleration_covariance = {0};

    /* Read IMU data */
    Mpu6050::Mpu6050_AccelData_t AccelData;
    Mpu6050::Mpu6050_GyroData_t GyroData;

    const auto read_status = mpu6050_dev_->Mpu6050_GetImuData(AccelData, GyroData);

    if (Mpu6050Hal::MPU6050_OK != read_status) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to read MPU6050 data");
        return;
    }

    const double accel_x = AccelData.Accel_X;
    const double accel_y = AccelData.Accel_Y;
    const double accel_z = AccelData.Accel_Z;
    const double gyro_x = (GyroData.Gyro_X - gyro_x_offset_) * (M_PI / 180.0);
    const double gyro_y = (GyroData.Gyro_Y - gyro_y_offset_) * (M_PI / 180.0);
    const double gyro_z = (GyroData.Gyro_Z - gyro_z_offset_) * (M_PI / 180.0);

    message.linear_acceleration.x = accel_x;
    message.linear_acceleration.y = accel_y;
    message.linear_acceleration.z = accel_z;
    message.angular_velocity_covariance[0] = 0.0;
    message.angular_velocity.x = gyro_x;
    message.angular_velocity.y = gyro_y;
    message.angular_velocity.z = gyro_z;

    const auto accelerometer_tilt = ros2_mpu6050::orientation::calculate_accelerometer_tilt(accel_x, accel_y, accel_z);

    ros2_mpu6050::orientation::EulerAngles orientation {roll_, pitch_, yaw_};

    if (!orientation_initialized_) {
        ros2_mpu6050::orientation::initialize_orientation(orientation, accelerometer_tilt);
        orientation_initialized_ = true;
    } else {
        const double dt = (current_time - last_update_time_).seconds();

        if (dt > 0.0 && dt < 1.0) {
            ros2_mpu6050::orientation::update_complementary_filter(orientation,
                                                                    accelerometer_tilt,
                                                                    gyro_x,
                                                                    gyro_y,
                                                                    gyro_z,
                                                                    dt,
                                                                    complementary_filter_alpha_);
        }
    }

    roll_ = orientation.roll;
    pitch_ = orientation.pitch;
    yaw_ = orientation.yaw;
    last_update_time_ = current_time;

    const auto quaternion = ros2_mpu6050::orientation::euler_to_quaternion(orientation);

    message.orientation.x = quaternion.x;
    message.orientation.y = quaternion.y;
    message.orientation.z = quaternion.z;
    message.orientation.w = quaternion.w;

    message.orientation_covariance = {
        0.02, 0.0, 0.0,
        0.0, 0.02, 0.0,
        0.0, 0.0, 999.0};

    publisher_->publish(message);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mpu6050Node>("mpu6050publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}