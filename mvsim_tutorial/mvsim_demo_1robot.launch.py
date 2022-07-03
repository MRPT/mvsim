
# ROS2 launch file

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mvsim',
            executable='mvsim_node',
            name='mvsim',
            output='screen',
            parameters=[
                get_package_share_directory(
                    "mvsim") + '/mvsim_tutorial/mvsim_demo_1robot.yaml'
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', [os.path.join(get_package_share_directory('mvsim'), 'mvsim_tutorial', 'mvsim_demo_1robot.rviz.rviz')]]
        )
    ])
