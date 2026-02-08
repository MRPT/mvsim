# ROS2 launch file for two robots demo world

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Get mvsim package directory
    mvsim_dir = get_package_share_directory('mvsim')

    # Launch configuration variables (no defaults here)
    world_file = LaunchConfiguration('world_file')
    do_fake_localization = LaunchConfiguration('do_fake_localization')
    publish_log_topics = LaunchConfiguration('publish_log_topics')

    # Declare launch arguments (defaults defined here)
    world_file_launch_arg = DeclareLaunchArgument(
        'world_file',
        default_value=TextSubstitution(
            text=os.path.join(mvsim_dir, 'mvsim_tutorial', 'demo_2robots.world.xml')
        ),
        description='Path to world XML file'
    )
    do_fake_localization_arg = DeclareLaunchArgument(
        'do_fake_localization',
        default_value='True',
        description='Publish fake identity tf "map" -> "odom"'
    )
    publish_log_topics_arg = DeclareLaunchArgument(
        'publish_log_topics',
        default_value='False',
        description='Publish every CSV-logger column as a std_msgs/Float64 topic per vehicle. '
                    'High-rate, disabled by default.'
    )

    # MVSim node
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            os.path.join(mvsim_dir, 'mvsim_tutorial', 'mvsim_ros2_params.yaml'),
            {
                'world_file': world_file,
                'do_fake_localization': do_fake_localization,
                'publish_log_topics': publish_log_topics,
            }
        ]
    )

    # RViz2 node for robot r1
    rviz2_r1_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='r1',
        arguments=[
            '-d',
            os.path.join(mvsim_dir, 'mvsim_tutorial', 'demo_2robots_r1_ros2.rviz')
        ],
        output='screen',
        remappings=[
            ('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')
        ]
    )

    # RViz2 node for robot r2
    rviz2_r2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='r2',
        arguments=[
            '-d',
            os.path.join(mvsim_dir, 'mvsim_tutorial', 'demo_2robots_r2_ros2.rviz')
        ],
        output='screen',
        remappings=[
            ('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')
        ]
    )

    return LaunchDescription([
        world_file_launch_arg,
        do_fake_localization_arg,
        publish_log_topics_arg,
        mvsim_node,
        rviz2_r1_node,
        rviz2_r2_node
    ])
