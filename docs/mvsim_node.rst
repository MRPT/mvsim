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
              world_file:=/path/to/your/my.world.xml \
              headless:=True

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim launch_world.launch.py \
              world_file:=/path/to/your/my.world.xml \
              headless:=True

|

All launch parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
All these parameters apply to both, ROS 1 and ROS 2 launch files above:

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

      'disable_sim_time_clock':
         Do not use the internal simulation clock: do not publish '/clock' and
         stamp all messages with wall-clock time instead, as done before
         simulation time support was added
         (default: 'False')

      'realtime_factor':
         Simulation speed: <1 slower, >1 faster than real time, 0 as fast as
         possible. (ROS 2 only)
         (default: '1.0')

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

        .. code-block:: xml

            <launch>
               <!-- Arguments -->
               <arg name="world_file" default="$(find my_package)/mvsim/demo.world.xml" />
               <arg name="headless" default="False" />
               <arg name="do_fake_localization" default="True" />
               <arg name="publish_tf_odom2baselink" default="True" />
               <arg name="force_publish_vehicle_namespace" default="False" />
               <arg name="disable_sim_time_clock" default="False" />
               <arg name="use_rviz" default="True" />
               <arg name="rviz_config_file" default="$(find mvsim)/mvsim_tutorial/demo_depth_camera.rviz" />

               <!-- Include the original launch file -->
               <include file="$(find mvsim)/launch/launch_world.launch">
                  <arg name="world_file" value="$(arg world_file)" />
                  <arg name="headless" value="$(arg headless)" />
                  <arg name="do_fake_localization" value="$(arg do_fake_localization)" />
                  <arg name="publish_tf_odom2baselink" value="$(arg publish_tf_odom2baselink)" />
                  <arg name="force_publish_vehicle_namespace" value="$(arg force_publish_vehicle_namespace)" />
                  <arg name="disable_sim_time_clock" value="$(arg disable_sim_time_clock)" />
                  <arg name="use_rviz" value="$(arg use_rviz)" />
                  <arg name="rviz_config_file" value="$(arg rviz_config_file)" />
               </include>
            </launch>

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
               disable_sim_time_clock = 'False'
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
                        'disable_sim_time_clock': disable_sim_time_clock,
                        'use_rviz': use_rviz,
                        'rviz_config_file': rviz_config_file
                  }.items()
               ))

               return ld


|

Simulation time and the ``/clock`` topic
-----------------------------------------

The ``mvsim_node`` acts as the ROS **time source** for the whole system:

* It publishes the global ``/clock`` topic (``rosgraph_msgs/Clock``) with the
  current simulation time (the wall-clock time at simulation start plus the
  elapsed simulated seconds).
* Every message header it publishes (``/odom``, ``/base_pose_ground_truth``,
  ``/tf``, and all sensor topics) is stamped with that same **simulation time**.
  Sensor messages in particular are stamped with the exact instant the
  observation was generated inside the simulation, so their timestamps are
  unaffected by any latency in the asynchronous ROS publisher threads.

Because header stamps track *simulation* time rather than wall-clock time, they
stay coherent even when the simulation cannot keep up with real time (real-time
factor below 1.0 due to heavy sensor/GUI load).

.. note::
   Run your **downstream** nodes with ``use_sim_time:=true`` so they consume
   ``/clock``. The ``mvsim_node`` itself *drives* the clock and normally runs
   with ``use_sim_time:=false``; setting ``use_sim_time:=true`` on the mvsim
   node is discouraged (a warning is printed) since it would make the node
   depend on the very clock it publishes.

Disabling the simulation clock
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you prefer the previous behavior, set the ``disable_sim_time_clock``
parameter (ROS launch argument) to ``true``. In that case:

* ``mvsim_node`` does not publish ``/clock``.
* All header stamps, including sensor observations, use plain wall-clock time
  (as reported by the ROS clock), regardless of the real-time factor.

.. code-block:: bash

   ros2 launch mvsim launch_world.launch.py \
     world_file:=/path/to/your/my.world.xml \
     disable_sim_time_clock:=True

|

Performance diagnostics
-------------------------
MVSim measures its own performance over consecutive windows of 2 seconds of simulated time:
achieved real time factor, wall-clock time spent in physics and waiting for OpenGL-based sensors,
and, per sensor, its processing time and achieved output rate.

- In the GUI: hover the mouse over the real time factor in the bottom status bar.
- ROS 2: published as ``diagnostic_msgs/DiagnosticArray`` on ``/diagnostics`` (status name
  ``mvsim: performance``), at ``diagnostics_rate`` Hz (node parameter, default ``1.0``,
  ``0`` disables it). Times are given as a percentage of the simulated time, so ``100%``
  means as much time as real time. The status is ``WARN`` if the achieved real time factor is
  below 90% of the requested one (``realtime_factor``).
  This publisher is only built if ``diagnostic_msgs`` is found.
- C++: ``World::getPerformanceStats()``.

Example (ROS 2): publish at 2 Hz and watch the values:

.. code-block:: bash

   ros2 run mvsim mvsim_node --ros-args \
     -p world_file:=/path/to/my.world.xml \
     -p diagnostics_rate:=2.0

   # In another terminal:
   ros2 topic echo /diagnostics
   # ...or with a GUI: ros2 run rqt_runtime_monitor rqt_runtime_monitor

Each message holds the keys ``realtime_factor``, ``realtime_factor_requested``,
``window_simul_time``, ``physics_steps``, ``physics_time_percent``,
``sensors_wait_time_percent``, and, for each sensor, ``<vehicle>/<sensor>/time_percent`` and
``<vehicle>/<sensor>/rate_hz``.

Example (C++):

.. code-block:: cpp

   const mvsim::World::PerformanceStats st = world.getPerformanceStats();
   std::cout << "RTF: " << st.realtime_factor << "\n";
   for (const auto& [name, s] : st.sensors)
   {
       std::cout << name << ": " << s.observations / st.window_simul_time << " Hz\n";
   }
