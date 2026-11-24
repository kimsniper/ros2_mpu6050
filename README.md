# MPU6050 ROS 2 Driver

## Overview

This package provides a ROS 2 driver for the MPU6050 accelerometer and gyroscope. It publishes acceleration, angular velocity, and estimated orientation using `sensor_msgs/msg/Imu`.

The driver was tested on a Raspberry Pi 4 running ROS 2 Humble.

## Hardware

- Raspberry Pi 4
- MPU6050 connected through I2C
- Default I2C address: `0x68`
- Default I2C bus: `/dev/i2c-1`

Verify the sensor connection:

```bash
i2cdetect -y 1
```

The output should show address `68`.

## Build

Clone the repository into the `src` directory of a ROS 2 workspace, then run:

```bash
colcon build --packages-select ros2_mpu6050
source install/setup.bash
```

## Run

Keep the robot and MPU6050 completely stationary while the node starts:

```bash
ros2 launch ros2_mpu6050 ros2_mpu6050.launch.py
```

The node performs stationary gyro calibration before publishing. It discards 100 warm-up samples and then measures 500 coherent IMU samples at 100 Hz. Publication begins only after calibration passes.

If startup calibration fails, keep the robot stationary and restart the node.

## Published Topic

Topic: `/imu/mpu6050`

Message type: `sensor_msgs/msg/Imu`

Inspect the published data:

```bash
ros2 topic echo /imu/mpu6050
```

Check the publication rate:

```bash
ros2 topic hz /imu/mpu6050
```

The default publication rate is 100 Hz. The default frame is `imu_link`.

## Orientation

Orientation is estimated with a complementary filter:

- Roll and pitch combine integrated gyroscope data with accelerometer tilt.
- Yaw is relative and is integrated from the calibrated Z-axis gyroscope.
- Relative yaw starts at zero when the node starts.
- The MPU6050 has no magnetometer, so yaw has no absolute heading reference and can drift.
- Yaw covariance is set to `999.0` to indicate high uncertainty.
- Published quaternions are normalized.

Do not use MPU6050 yaw as the only absolute heading source for navigation.

## Startup Calibration

Startup calibration is part of normal node initialization. Calibration values remain in memory and are not stored in `params.yaml`.

Calibration checks the following:

- Acceleration magnitude is consistent with stationary gravity.
- Acceleration-norm standard deviation does not exceed `0.15 m/s²`.
- Gyroscope standard deviation does not exceed `0.5 deg/s` on any axis.
- All measurements and calculated statistics are finite.

The optional diagnostic executable runs the same calibration algorithm:

```bash
ros2 run ros2_mpu6050 ros2_mpu6050_calibrate
```

Its output is for inspection only. It is not required before starting the node and must not be copied into `params.yaml`.

## Sensor Configuration

Default settings in `config/params.yaml`:

- Gyroscope range: `±250 deg/s`
- Accelerometer range: `±2 g`
- Digital low-pass filter: `DLPF_CFG=3`
- Accelerometer bandwidth: approximately 44 Hz
- Gyroscope bandwidth: approximately 42 Hz
- Hardware sample rate: `1000 / (1 + 9) = 100 Hz`
- Clock source: X-axis gyroscope PLL
- Complementary-filter coefficient: `0.98`
- Published frame: `imu_link`

## Tests

Run all package tests:

```bash
colcon test --packages-select ros2_mpu6050
colcon test-result --verbose
```
