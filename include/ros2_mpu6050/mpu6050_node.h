#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H

#include "ros2_mpu6050/mpu6050.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Mpu6050Node : public rclcpp::Node {
 public:
  Mpu6050Node(const std::string& name);

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<Mpu6050> mpu6050_dev_;
  rclcpp::TimerBase::SharedPtr timer_;
  void ImuPubCallback();

  double gyro_x_offset_ {0.0};
  double gyro_y_offset_ {0.0};
  double gyro_z_offset_ {0.0};
  double accel_x_offset_ {0.0};
  double accel_y_offset_ {0.0};
  double accel_z_offset_ {0.0};

};

#endif  // MPU6050DRIVER_H