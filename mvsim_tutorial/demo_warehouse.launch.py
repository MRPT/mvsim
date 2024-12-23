# ROS2 launch file, invoking mvsim/launch/launch_world.launch.py
# See: https://mvsimulator.readthedocs.io/en/latest/mvsim_node.html

from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # mvsim directory:
    mvsimDir = get_package_share_directory("mvsim")

    # World to launch:
    world_file = os.path.join(
        mvsimDir, 'mvsim_tutorial', 'demo_warehouse.world.xml')
    rviz_config_file = os.path.join(
        get_package_share_directory('mvsim'), 'mvsim_tutorial', 'demo_warehouse_ros2.rviz')

    # Configurable arguments
    headless = LaunchConfiguration('headless', default='True')
    do_fake_localization = LaunchConfiguration(
        'do_fake_localization', default='False')
    publish_tf_odom2baselink = LaunchConfiguration(
        'publish_tf_odom2baselink', default='False')
    force_publish_vehicle_namespace = LaunchConfiguration(
        'force_publish_vehicle_namespace', default='True')
    use_rviz = LaunchConfiguration('use_rviz', default='False')

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add arguments to LaunchDescription
    ld.add_action(DeclareLaunchArgument('headless', default_value='False',
                                        description='Run in headless mode'))
    ld.add_action(DeclareLaunchArgument('do_fake_localization', default_value='True',
                                        description='Publish fake identity tf "map" -> "odom"'))
    ld.add_action(DeclareLaunchArgument('publish_tf_odom2baselink', default_value='True',
                                        description='Publish tf "odom" -> "base_link"'))
    ld.add_action(DeclareLaunchArgument('force_publish_vehicle_namespace', default_value='False',
                                        description='Use vehicle name namespace even if there is only one vehicle'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True',
                                        description='Whether to launch RViz2'))

    # Include the original launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mvsim'),
                         'launch', 'launch_world.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'headless': headless,
            'do_fake_localization': do_fake_localization,
            'publish_tf_odom2baselink': publish_tf_odom2baselink,
            'force_publish_vehicle_namespace': force_publish_vehicle_namespace,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file
        }.items()
    ))

    return ld
