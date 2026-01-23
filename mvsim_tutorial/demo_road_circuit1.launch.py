# ROS2 launch file for road circuit demo world

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Get mvsim package directory
    mvsim_dir = get_package_share_directory('mvsim')

    # Launch configuration variables (no defaults here)
    world_file = LaunchConfiguration('world_file')
    headless = LaunchConfiguration('headless')
    do_fake_localization = LaunchConfiguration('do_fake_localization')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments (defaults defined here)
    world_file_launch_arg = DeclareLaunchArgument(
        'world_file',
        default_value=TextSubstitution(
            text=os.path.join(mvsim_dir, 'mvsim_tutorial', 'demo_road_circuit1.world.xml')
        ),
        description='Path to world XML file'
    )
    headless_launch_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run in headless mode'
    )
    do_fake_localization_arg = DeclareLaunchArgument(
        'do_fake_localization',
        default_value='True',
        description='Publish tf odom -> base_link'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz2'
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
                'headless': headless,
                'do_fake_localization': do_fake_localization,
            }
        ]
    )

    # RViz2 nodes for each robot
    rviz2_nodes = [
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                os.path.join(mvsim_dir, 'mvsim_tutorial', 'demo_road_circuit1_ros2.rviz')
            ],
            output='screen',
            remappings=[
                ('/map', f'/r{idx}/map'),
                ('/tf', f'/r{idx}/tf'),
                ('/tf_static', f'/r{idx}/tf_static'),
                ('/goal_pose', f'/r{idx}/goal_pose'),
                ('/clicked_point', f'/r{idx}/clicked_point'),
                ('/vehs/cmd_vel', f'/r{idx}/cmd_vel'),
                ('/vehs/lidar1_points', f'/r{idx}/lidar1_points'),
                ('/vehs/camera1/image_raw', f'/r{idx}/camera1/image_raw'),
                ('/vehs/chassis_markers', f'/r{idx}/chassis_markers'),
                ('/vehs/scanner1', f'/r{idx}/scanner1'),
            ],
        )
        for idx in range(1, 2)
    ]

    return LaunchDescription([
        world_file_launch_arg,
        headless_launch_arg,
        do_fake_localization_arg,
        use_rviz_arg,
        mvsim_node
    ] + rviz2_nodes)
