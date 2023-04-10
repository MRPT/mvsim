# ROS2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
import os

mvsimDir = get_package_share_directory("mvsim")

MVSIM_WORLD_FILE = os.path.join(mvsimDir, 'mvsim_tutorial',
                                'demo_turtlebot_world.world.xml')
MVSIM_ROS2_PARAMS_FILE = os.path.join(mvsimDir, 'mvsim_tutorial',
                                      'mvsim_ros2_params.yaml')
RVIZ2_FILE = os.path.join(mvsimDir, 'mvsim_tutorial',
                          'demo_turtlebot_world_ros2.rviz')


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", default_value=TextSubstitution(
            text=MVSIM_WORLD_FILE))

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            MVSIM_ROS2_PARAMS_FILE,
            {
                "world_file": LaunchConfiguration('world_file'),
                "headless": False
            }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [RVIZ2_FILE]]
    )

    return LaunchDescription([
        world_file_launch_arg,
        mvsim_node,
        rviz2_node
    ])
