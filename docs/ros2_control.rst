.. _ros2_control:

ros2_control integration
==========================

MVSim vehicles can be driven by standard `ros2_control <https://control.ros.org/>`_
controllers (e.g. ``diff_drive_controller``), as with ``gz_ros2_control`` in Gazebo, so a
ROS 2 stack can switch between a real robot, Gazebo and MVSim without changes above the
hardware layer: same controllers, topics and frames.

.. note::
   Requires the ROS 2 node (``mvsim_node``) built with ``controller_manager`` and
   ``hardware_interface`` (>= 6.0) available. Currently supported for ``differential``,
   ``differential_3_wheels`` and ``differential_4_wheels`` (skid steer) vehicles.

|

How it works
---------------

- The vehicle uses ``<controller class="ros2_control">``. MVSim then exposes one
  joint per wheel, with ``position`` (continuous), ``velocity`` and ``effort`` state
  interfaces, and a ``velocity`` or ``effort`` command interface.
- With ``velocity`` commands, an inner per-wheel PID (``KP``, ``KI``, ``KD``,
  ``max_torque``, same units than the ``twist_pid`` controller) produces the wheel
  torques, so wheels may lag and slip realistically. With ``effort`` commands, torques
  are applied directly.
- ``mvsim_node`` creates one controller manager per such vehicle, named
  ``controller_manager``, in the vehicle namespace (only if there are several vehicles or
  ``force_publish_vehicle_namespace`` is set).
- **Lock-step:** the controller manager ``read()``/``update()``/``write()`` cycle runs in
  the simulation thread at its ``update_rate`` (a multiple of the simulation time step,
  ``<simul_timestep>``), stamped with simulation time. Controllers see the same timing
  regardless of the real time factor or the CPU load.
- For these vehicles, ``mvsim_node`` does not subscribe to ``cmd_vel`` nor publishes
  odometry (topic and ``odom -> base_link`` TF) or the fake localization
  (``map -> odom``): those come from the controllers. Ground truth and sensors are
  unchanged.

|

Vehicle XML
-------------

.. code-block:: xml

    <dynamics class="differential_4_wheels">
      <lf_wheel pos="0.13  0.16" diameter="0.20" ... />  <!-- joint: "lf_wheel_joint" -->
      <rf_wheel pos="0.13 -0.16" diameter="0.20" joint_name="my_rf_joint" ... />
      ...
      <controller class="ros2_control">
        <command_interface>velocity</command_interface>  <!-- or "effort" -->
        <KP>18.3</KP> <KI>474.5</KI> <KD>0</KD> <max_torque>100</max_torque>
        <controllers_yaml>ros2_control/jackal_controllers.yaml</controllers_yaml>
        <robot_description>generated</robot_description>  <!-- or "topic" -->
      </controller>
    </dynamics>

- **Joint names:** the ``joint_name`` attribute of each wheel, or the wheel tag name
  plus ``_joint`` by default (e.g. ``l_wheel_joint``, ``lf_wheel_joint``).
- ``controllers_yaml``: parameters for the controller manager (``update_rate``, the
  controllers and their types) and for the controllers, as in any ros2_control setup.
  Relative paths are relative to the world file. Each controller declared there with
  ``<name>.type`` reads its parameters from the same file, unless ``<name>.params_file``
  is given. Controllers should also set ``use_sim_time: true``.
- ``robot_description``:

  - ``generated`` (default): MVSim publishes a URDF on ``robot_description`` (in the
    vehicle namespace) with ``base_link``, one ``continuous`` joint per wheel and the
    ``<ros2_control>`` block. No URDF or xacro needed.
  - ``topic``: the robot description is read from the ``robot_description`` topic, e.g.
    published by ``robot_state_publisher`` from your existing URDF. Every ``<ros2_control>``
    system whose joints are all wheel joints of the vehicle is bound to the simulated
    wheels, **whatever its hardware plugin is** (e.g. ``gz_ros2_control/GazeboSimSystem``
    or a real hardware driver), so the same URDF serves real hardware, Gazebo and MVSim.
    Other ``<ros2_control>`` blocks are ignored with a warning. The command interfaces
    in the URDF must match ``<command_interface>``.

The URDF is only used for control and frames: MVSim physics always come from its own
vehicle XML (wheel positions, diameters, masses, friction), which must be kept consistent
with the URDF.

|

Example
---------

``mvsim_tutorial/demo_ros2_control.world.xml`` has a skid-steer robot (four velocity
controlled wheels, no ``mimic`` joints) driven by ``diff_drive_controller``, with
``joint_state_broadcaster``:

.. code-block:: bash

    # MVSim-generated robot description:
    ros2 launch mvsim demo_ros2_control.launch.py

    # Or, with an existing URDF whose <ros2_control> block targets Gazebo
    # (mvsim_tutorial/ros2_control/jackal.urdf), published by robot_state_publisher:
    ros2 launch mvsim demo_ros2_control.launch.py robot_description:=urdf

    # Drive it:
    ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
      "{twist: {linear: {x: 0.5}, angular: {z: 0.3}}}"

Odometry is published by ``diff_drive_controller`` (``/diff_drive_controller/odom`` and the
``odom -> base_link`` TF), and MVSim ground truth in ``/base_pose_ground_truth``.

|

Simulation rate vs. controller rate
-------------------------------------

The controller manager ``update_rate`` should be an integer multiple of the simulation rate
(``1/simul_timestep``); otherwise, the closest multiple is used and a warning is printed.
For instance, ``simul_timestep=0.005`` (200 Hz) with ``update_rate: 100`` updates the
controllers every 2 physics steps. Smaller simulation time steps improve the accuracy of the
wheel-ground contact dynamics at a higher CPU cost; 2 to 5 physics steps per controller update
is a good default.
