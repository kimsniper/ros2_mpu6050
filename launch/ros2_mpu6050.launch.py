import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('ros2_mpu6050')

    param_file = LaunchConfiguration('param_file')
    i2c_address = LaunchConfiguration('i2c_address')

    params_arg = DeclareLaunchArgument('param_file',
                                        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
                                        description='Path to the ROS2 parameter file')

    i2c_address_arg = DeclareLaunchArgument('i2c_address',
                                             default_value='104',
                                             description='I2C address of the MPU6050 (104=0x68 when AD0 low, 105=0x69 when AD0 high)')

    mpu6050_sensor = Node(
        package='ros2_mpu6050',
        executable='ros2_mpu6050',
        name='mpu6050_sensor',
        output="screen",
        emulate_tty=True,
        parameters=[param_file,
                    {'i2c_address': i2c_address}]
    )

    return LaunchDescription([
        params_arg,
        i2c_address_arg,
        mpu6050_sensor
    ])
