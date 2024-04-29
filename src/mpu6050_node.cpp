#include "ros2_mpu6050/mpu6050_node.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node(const std::string& name)
    : Node(name)
    , mpu6050_dev_{std::make_unique<Mpu6050>()}
{
    // Declare parameters
    this->declare_parameter<int>("gyro_fs_sel", 0);
    this->declare_parameter<int>("accel_afs_sel", 0);
    this->declare_parameter<int>("dlpf_cfg", 0);
    this->declare_parameter<int>("clock_src", 0);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);

    /* Assign offset values */
    gyro_x_offset_ = this->get_parameter("gyro_x_offset").as_double();
    gyro_y_offset_ = this->get_parameter("gyro_y_offset").as_double();
    gyro_z_offset_ = this->get_parameter("gyro_z_offset").as_double();
    accel_x_offset_ = this->get_parameter("accel_x_offset").as_double();
    accel_y_offset_ = this->get_parameter("accel_y_offset").as_double();
    accel_z_offset_ = this->get_parameter("accel_z_offset").as_double();

    /* Assign sensor paramaters */
    mpu6050_dev_->Mpu6050_GyroFsSel(static_cast<Mpu6050::Mpu6050_FsSel_t>(this->get_parameter("gyro_fs_sel").as_int()));
    mpu6050_dev_->Mpu6050_AccelFsSel(static_cast<Mpu6050::Mpu6050_AfsSel_t>(this->get_parameter("accel_afs_sel").as_int()));
    mpu6050_dev_->Mpu6050_DlpfConfig(static_cast<Mpu6050::Mpu6050_DlpfCfg_t>(this->get_parameter("dlpf_cfg").as_int()));
    mpu6050_dev_->Mpu6050_ClockSelect(static_cast<Mpu6050::Mpu6050_ClkSrc_t>(this->get_parameter("clock_src").as_int()));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/mpu6050", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&Mpu6050Node::ImuPubCallback, this));
}

void Mpu6050Node::ImuPubCallback()
{
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    message.linear_acceleration_covariance = {0};

    /* Read IMU data */
    Mpu6050::Mpu6050_AccelData_t AccelData;
    Mpu6050::Mpu6050_GyroData_t GyroData;

    mpu6050_dev_->Mpu6050_GetAccelData(AccelData);
    mpu6050_dev_->Mpu6050_GetGyroData(GyroData);

    message.linear_acceleration.x = AccelData.Accel_X - accel_x_offset_;
    message.linear_acceleration.y = AccelData.Accel_Y - accel_y_offset_;
    message.linear_acceleration.z = AccelData.Accel_Z - accel_z_offset_;
    message.angular_velocity_covariance[0] = {0};
    message.angular_velocity.x = (GyroData.Gyro_X - gyro_x_offset_) * (M_PI / 180.0);
    message.angular_velocity.y = (GyroData.Gyro_Y - gyro_y_offset_) * (M_PI / 180.0);
    message.angular_velocity.z = (GyroData.Gyro_Z - gyro_z_offset_) * (M_PI / 180.0);
    
    message.orientation_covariance[0] = -1;
    message.orientation.x = 0;
    message.orientation.y = 0;
    message.orientation.z = 0;
    message.orientation.w = 0;
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