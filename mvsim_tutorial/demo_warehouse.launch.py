# ROS2 launch file for warehouse demo world
# Invokes mvsim/launch/launch_world.launch.py
# See: https://mvsimulator.readthedocs.io/en/latest/mvsim_node.html

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get mvsim package directory
    mvsim_dir = get_package_share_directory('mvsim')

    # World and configuration files
    world_file = os.path.join(
        mvsim_dir, 'mvsim_tutorial', 'demo_warehouse.world.xml'
    )
    rviz_config_file = os.path.join(
        mvsim_dir, 'mvsim_tutorial', 'demo_warehouse_ros2.rviz'
    )

    # Launch configuration variables (no defaults here)
    headless = LaunchConfiguration('headless')
    do_fake_localization = LaunchConfiguration('do_fake_localization')
    publish_tf_odom2baselink = LaunchConfiguration('publish_tf_odom2baselink')
    publish_log_topics = LaunchConfiguration('publish_log_topics')
    force_publish_vehicle_namespace = LaunchConfiguration(
        'force_publish_vehicle_namespace')
    use_rviz = LaunchConfiguration('use_rviz')

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments (defaults defined here)
    ld.add_action(
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run in headless mode'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'do_fake_localization',
            default_value='True',
            description='Publish fake identity tf "map" -> "odom"'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'publish_tf_odom2baselink',
            default_value='True',
            description='Publish tf "odom" -> "base_link"'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'force_publish_vehicle_namespace',
            default_value='False',
            description='Use vehicle name namespace even if there is only one vehicle'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to launch RViz2'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'publish_log_topics',
            default_value='False',
            description='Publish every CSV-logger column as a std_msgs/Float64 topic per vehicle. '
                        'High-rate, disabled by default.'
        )
    )

    # Include world launch file
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mvsim'),
                    'launch',
                    'launch_world.launch.py'
                )
            ),
            launch_arguments={
                'world_file': world_file,
                'headless': headless,
                'do_fake_localization': do_fake_localization,
                'publish_tf_odom2baselink': publish_tf_odom2baselink,
                'force_publish_vehicle_namespace': force_publish_vehicle_namespace,
                'publish_log_topics': publish_log_topics,
                'use_rviz': use_rviz,
                'rviz_config_file': rviz_config_file
            }.items()
        )
    )

    return ld
