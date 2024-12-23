Use from ROS
===================

MVSim comes with a ROS 1 and ROS 2 **node** (an executable named `mvsim_node`),
which is shipped with the **ROS package** named `mvsim`.

.. note::
   You can also use the cli application ``mvsim`` if you want to test or run your world without launching a whole ROS system.
   See :ref:`mvsim cli`.


Generic launch file
-------------------------
This launch file can be used to launch a simulated world integrated with ROS from the command line,
or can be also included into your own launch file by setting its **ROS launch arguments**.

|

Basic usage from the command line
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim launch_world.launch \
              XX

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim launch_world.launch.py \
              world_file:=/path/to/your/my.world.xml

|

All launch parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            xxx

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            $ ros2 launch mvsim launch_world.launch.py --show-args
            Arguments (pass arguments as '<name>:=<value>'):

               'world_file':
                  Path to the *.world.xml file to load

               'headless':
                  no description given
                  (default: 'False')

               'do_fake_localization':
                  publish fake identity tf "map" -> "odom"
                  (default: 'True')

               'publish_tf_odom2baselink':
                  publish tf "odom" -> "base_link"
                  (default: 'True')

               'force_publish_vehicle_namespace':
                  Use vehicle name namespace even if there is only one vehicle
                  (default: 'False')

               'use_rviz':
                  Whether to launch RViz2
                  (default: 'True')

               'rviz_config_file':
                  If use_rviz:="True", the configuration file for rviz
                  (default: '')

|

How to include it into your own launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You can use this code to bootstrap your own launch files:


.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            xxx

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: python

            import os
            from launch import LaunchDescription
            from launch.actions import IncludeLaunchDescription
            from launch.substitutions import LaunchConfiguration
            from launch.launch_description_sources import PythonLaunchDescriptionSource
            from ament_index_python.packages import get_package_share_directory

            def generate_launch_description():
               # *** REMEMBER: Change this to your actual world file ***
               world_file = os.path.join(
                  get_package_share_directory('my_package'), 'mvsim', 'demo.world.xml')

               # and replace this RViz file with yours as needed:
               rviz_config_file = os.path.join(
                  get_package_share_directory('mvsim'), 'mvsim_tutorial', 'demo_depth_camera_ros2.rviz')

               headless = 'False'
               do_fake_localization = 'True'
               publish_tf_odom2baselink = 'True'
               force_publish_vehicle_namespace = 'False'
               use_rviz = 'True'


               # Create LaunchDescription
               ld = LaunchDescription()

               # Add actions to LaunchDescription
               ld.add_action(IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('mvsim'), 'launch', 'launch_world.launch.py')
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


|
