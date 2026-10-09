# ROS 2 launch file: a skid-steer robot driven by ros2_control controllers
# (diff_drive_controller + joint_state_broadcaster), in lock-step with MVSim.
#
# Usage:
#   ros2 launch mvsim demo_ros2_control.launch.py
#   ros2 launch mvsim demo_ros2_control.launch.py robot_description:=urdf
#
# Then, drive it with TwistStamped messages on /diff_drive_controller/cmd_vel

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    mvsim_dir = get_package_share_directory('mvsim')
    tutorial_dir = os.path.join(mvsim_dir, 'mvsim_tutorial')

    robot_description_arg = DeclareLaunchArgument(
        'robot_description', default_value='generated',
        choices=['generated', 'urdf'],
        description='"generated": MVSim generates the robot description from the vehicle; '
                    '"urdf": use an existing URDF (published by robot_state_publisher), '
                    'whose <ros2_control> joints are bound to the simulated wheels')
    headless_arg = DeclareLaunchArgument('headless', default_value='False')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='True')

    # The world file reads this to choose where the robot description comes from:
    set_env = SetEnvironmentVariable(
        'MVSIM_ROBOT_DESCRIPTION', 'topic',
        condition=LaunchConfigurationEquals('robot_description', 'urdf'))

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[{
            'world_file': os.path.join(tutorial_dir, 'demo_ros2_control.world.xml'),
            'headless': LaunchConfiguration('headless'),
        }])

    with open(os.path.join(tutorial_dir, 'ros2_control', 'jackal.urdf'), 'r') as f:
        urdf = f.read()

    robot_state_publisher = Node(
        condition=LaunchConfigurationEquals('robot_description', 'urdf'),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}])

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_drive_controller'],
        parameters=[{'use_sim_time': True}])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(tutorial_dir, 'demo_ros2_control.rviz')])

    return LaunchDescription([
        robot_description_arg,
        headless_arg,
        use_rviz_arg,
        set_env,
        mvsim_node,
        robot_state_publisher,
        spawner,
        rviz2_node,
    ])
