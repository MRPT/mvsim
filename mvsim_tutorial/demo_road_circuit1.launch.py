# ROS2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
import os


def generate_launch_description():
    mvsimDir = get_package_share_directory("mvsim")
    # print('mvsimDir: ' + mvsimDir)

    # args that can be set from the command line or a default will be used
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", default_value=TextSubstitution(
            text=os.path.join(mvsimDir, 'mvsim_tutorial', 'demo_road_circuit1.world.xml')))

    headless_launch_arg = DeclareLaunchArgument(
        "headless", default_value='False')

    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization", default_value='True', description='publish tf odom -> base_link')

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            os.path.join(mvsimDir, 'mvsim_tutorial',
                         'mvsim_ros2_params.yaml'),
            {
                "world_file": LaunchConfiguration('world_file'),
                "headless": LaunchConfiguration('headless'),
                "do_fake_localization": LaunchConfiguration('do_fake_localization'),
            }]
    )

    # rviz
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    rviz2_nodes = [
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # namespace=f"veh{idx}",
            arguments=[
                '-d', [os.path.join(mvsimDir, 'mvsim_tutorial', 'demo_road_circuit1_ros2.rviz')]],
            output='screen',
            remappings=[
                ("/map", f"/r{idx}/map"),
                ("/tf", f"/r{idx}/tf"),
                ("/tf_static", f"/r{idx}/tf_static"),
                ("/goal_pose", f"/r{idx}/goal_pose"),
                ("/clicked_point", f"/r{idx}/clicked_point"),
                ("/vehs/cmd_vel", f"/r{idx}/cmd_vel"),
                ("/vehs/lidar1_points", f"/r{idx}/lidar1_points"),
                ("/vehs/camera1/image_raw", f"/r{idx}/camera1/image_raw"),
                ("/vehs/chassis_markers", f"/r{idx}/chassis_markers"),
                ("/vehs/scanner1", f"/r{idx}/scanner1"),
            ],
        )
        for idx in range(1, 2)
    ]

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [os.path.join(mvsimDir, 'mvsim_tutorial', 'demo_warehouse_ros2.rviz')]]
    )

    return LaunchDescription([
        world_file_launch_arg,
        declare_use_rviz_cmd,
        headless_launch_arg,
        do_fake_localization_arg,
        mvsim_node
    ]
        + rviz2_nodes
    )
