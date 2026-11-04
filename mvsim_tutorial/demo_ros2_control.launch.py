# ROS 2 launch file: a vehicle driven by ros2_control controllers, in
# lock-step with MVSim:
#  - vehicle:=jackal (default): skid-steer robot (differential_4_wheels),
#    with diff_drive_controller.
#  - vehicle:=car_ackermann: car-like vehicle with one motor per wheel (ackermann),
#    with ackermann_steering_controller.
#  - vehicle:=agricobiot2: car-like robot with a drivetrain
#    (ackermann_drivetrain), with bicycle_steering_controller.
#  - vehicle:=skid_steer_4wd: skid-steer robot whose vehicle definition is
#    generated from its URDF (mvsim-urdf2xml), with diff_drive_controller.
# All of them, with joint_state_broadcaster.
#
# Usage:
#   ros2 launch mvsim demo_ros2_control.launch.py [vehicle:=car_ackermann]
#   ros2 launch mvsim demo_ros2_control.launch.py [vehicle:=car_ackermann] robot_description:=urdf
#
# Then, drive it with TwistStamped messages on the topic printed at start up.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

# Per vehicle: world file, example URDF and the controller to drive it.
VEHICLES = {
    'jackal': {
        'world': 'demo_ros2_control.world.xml',
        'urdf': 'jackal.urdf',
        'controller': 'diff_drive_controller',
        'cmd_topic': '/diff_drive_controller/cmd_vel',
    },
    'car_ackermann': {
        'world': 'demo_ros2_control_ackermann.world.xml',
        'urdf': 'car_ackermann.urdf',
        'controller': 'ackermann_steering_controller',
        'cmd_topic': '/ackermann_steering_controller/reference',
    },
    'agricobiot2': {
        'world': 'demo_ros2_control_ackermann_drivetrain.world.xml',
        'urdf': 'agricobiot2.urdf',
        'controller': 'bicycle_steering_controller',
        'cmd_topic': '/bicycle_steering_controller/reference',
    },
    'skid_steer_4wd': {
        'world': 'demo_ros2_control_skid_steer.world.xml',
        'urdf': 'skid_steer_4wd.urdf',
        'controller': 'diff_drive_controller',
        'cmd_topic': '/diff_drive_controller/cmd_vel',
    },
}


def launch_setup(context):
    tutorial_dir = os.path.join(get_package_share_directory('mvsim'), 'mvsim_tutorial')
    veh = VEHICLES[LaunchConfiguration('vehicle').perform(context)]
    use_urdf = LaunchConfiguration('robot_description').perform(context) == 'urdf'

    # The world files read this to choose where the robot description comes from:
    set_env = SetEnvironmentVariable(
        'MVSIM_ROBOT_DESCRIPTION', 'topic' if use_urdf else 'generated')

    # Note: no 'name' (the default one is 'mvsim'), since in ROS 2 Humble it
    # would also rename the controller nodes, which then miss their parameters.
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        output='screen',
        parameters=[{
            'world_file': os.path.join(tutorial_dir, veh['world']),
            'headless': LaunchConfiguration('headless'),
            # With a URDF, robot_state_publisher publishes the sensor frames:
            'publish_sensor_tf': not use_urdf,
        }])

    actions = [set_env, mvsim_node]
    if use_urdf:
        with open(os.path.join(tutorial_dir, 'ros2_control', veh['urdf']), 'r') as f:
            urdf = f.read()
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf, 'use_sim_time': True}]))

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', veh['controller']])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(tutorial_dir, 'demo_ros2_control.rviz')])

    info = LogInfo(msg='Drive it with geometry_msgs/msg/TwistStamped messages on ' +
                   veh['cmd_topic'])

    return actions + [spawner, rviz2_node, info]


def generate_launch_description():
    vehicle_arg = DeclareLaunchArgument(
        'vehicle', default_value='jackal', choices=list(VEHICLES.keys()),
        description='Vehicle (and controller) of the demo')
    robot_description_arg = DeclareLaunchArgument(
        'robot_description', default_value='generated',
        choices=['generated', 'urdf'],
        description='"generated": MVSim generates the robot description from the vehicle; '
                    '"urdf": use an existing URDF (published by robot_state_publisher), '
                    'whose <ros2_control> joints are bound to the simulated ones')
    headless_arg = DeclareLaunchArgument('headless', default_value='False')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='True')

    return LaunchDescription([
        vehicle_arg,
        robot_description_arg,
        headless_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
