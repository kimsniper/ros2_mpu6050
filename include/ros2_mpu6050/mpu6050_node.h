#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H

#include "ros2_mpu6050/mpu6050.h"
#include "ros2_mpu6050/orientation_math.h"
#include "ros2_mpu6050/startup_calibration.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
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

  double complementary_filter_alpha_ {0.98};
  std::string frame_id_ {"imu_link"};
  double roll_ {0.0};
  double pitch_ {0.0};
  double yaw_ {0.0};
  rclcpp::Time last_update_time_;
  bool orientation_initialized_ {false};

};

#endif  // MPU6050DRIVER_H