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

    params_arg = DeclareLaunchArgument('param_file',
                                        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
                                        description='Path to the ROS2 parameter file')

    mpu6050_sensor = Node(
        package='ros2_mpu6050',
        executable='ros2_mpu6050',
        name='mpu6050_sensor',
        output="screen",
        emulate_tty=True,
        parameters=[param_file]
    )

    return LaunchDescription([
        params_arg,
        mpu6050_sensor
    ])
