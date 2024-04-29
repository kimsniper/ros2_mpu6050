cmake_minimum_required(VERSION 3.8)
project(ros2_mpu6050)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(ros2_mpu6050 src/mpu6050.cpp src/mpu6050_hal.cpp src/mpu6050_node.cpp)
ament_target_dependencies(ros2_mpu6050 rclcpp sensor_msgs)
target_link_libraries(ros2_mpu6050 i2c)

add_executable(ros2_mpu6050_calibrate src/mpu6050.cpp src/mpu6050_hal.cpp src/mpu6050_calibrate.cpp)
ament_target_dependencies(ros2_mpu6050_calibrate)
target_link_libraries(ros2_mpu6050_calibrate i2c)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  ros2_mpu6050
  ros2_mpu6050_calibrate
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
