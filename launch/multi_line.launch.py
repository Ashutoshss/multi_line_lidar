from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ydlidar_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )


    rviz_config = os.path.join(
        get_package_share_directory('multi_line_lidar'),
        'rviz',
        'default_view.rviz'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch_path)
        ),

        Node(
            package='multi_line_lidar',
            executable='Scanner.py',
            name='multi_line_lidar',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
