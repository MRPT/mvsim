.. _ros2_control:

ros2_control integration
==========================

MVSim vehicles can be driven by standard `ros2_control <https://control.ros.org/>`_
controllers (e.g. ``diff_drive_controller``, ``ackermann_steering_controller``,
``bicycle_steering_controller``), as with ``gz_ros2_control`` in Gazebo, so a
ROS 2 stack can switch between a real robot, Gazebo and MVSim without changes above the
hardware layer: same controllers, topics and frames.

.. note::
   Requires the ROS 2 node (``mvsim_node``) built with ``controller_manager`` and
   ``hardware_interface`` available (ROS 2 Humble, Jazzy and newer). Supported for
   ``differential``, ``differential_3_wheels``, ``differential_4_wheels`` (skid steer),
   ``ackermann`` and ``ackermann_drivetrain`` vehicles (see :ref:`ros2_control_ackermann`).
   Vehicles with other controllers, and MVSim without ROS, are not affected.

|

How it works
---------------

- The vehicle uses ``<controller class="ros2_control">``. MVSim then exposes one
  joint per wheel, with ``position`` (continuous), ``velocity`` and ``effort`` state
  interfaces, and a ``velocity`` or ``effort`` command interface (Ackermann vehicles
  also have steering joints, see below).
- With ``velocity`` commands, an inner per-wheel PID (``KP``, ``KI``, ``KD``,
  ``max_torque``, same units as the ``twist_pid`` controller) produces the wheel
  torques, so wheels may lag and slip realistically. A zero velocity command brakes
  with the proportional term only, so the vehicle stops without rebounding. With
  ``effort`` commands, torques are applied directly. Wheels without a command (``NaN``,
  e.g. a caster joint not claimed by any controller, or without a command interface in
  the URDF) roll freely.
- ``mvsim_node`` creates one controller manager per such vehicle, named
  ``controller_manager``, in the vehicle namespace (only if there are several vehicles or
  ``force_publish_vehicle_namespace`` is set).
- **Lock-step:** the controller manager ``read()``/``update()``/``write()`` cycle runs in
  the simulation thread at its ``update_rate`` (a multiple of the simulation time step,
  ``<simul_timestep>``), stamped with simulation time. Controllers see the same timing
  regardless of the real time factor or the CPU load.
- For these vehicles, ``mvsim_node`` neither subscribes to ``cmd_vel`` nor publishes
  odometry (topic and ``odom -> base_link`` TF, or the vehicle ``<base_frame_id>``) or the fake localization
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
- **ROS 2 Humble:** do not set the node name of ``mvsim_node`` (e.g. ``name=`` in a launch
  file ``Node`` action; its default name is ``mvsim``), since in Humble that remap renames
  the controller nodes too, so they do not get their parameters.
- ``controllers_yaml``: parameters for the controller manager (``update_rate``, the
  controllers and their types) and for the controllers, as in any ros2_control setup.
  Relative paths are relative to the world file. Each controller declared there with
  ``<name>.type`` reads its parameters from the same file, unless ``<name>.params_file``
  is given. Controllers should also set ``use_sim_time: true``.
- **Vehicles inserted at runtime** (see :ref:`dynamic-worlds`) get their controller
  manager too, always in their namespace, and it is destroyed when they are removed.
- **Several vehicles:** each one gets its own controller manager in its namespace
  (``/<vehicle>/controller_manager``), and its controllers are spawned with
  ``--controller-manager /<vehicle>/controller_manager``. Prefix node names in the YAML
  file with ``/**/`` to share the same parameters among all of them (as in the tutorial
  file), or with ``/<vehicle>/`` for per-vehicle values.
- ``robot_description``:

  - ``generated`` (default): MVSim publishes a URDF on ``robot_description`` (in the
    vehicle namespace) with the vehicle frame (``base_link``, or its ``<base_frame_id>``),
    one ``continuous`` joint per wheel and the
    ``<ros2_control>`` block. No URDF or xacro needed.
  - ``topic``: the robot description is read from the ``robot_description`` topic, e.g.
    published by ``robot_state_publisher`` from your existing URDF. Every ``<ros2_control>``
    system whose joints are all joints of the vehicle is bound to the simulated ones,
    **whatever its hardware plugin is** (e.g. ``gz_ros2_control/GazeboSimSystem``
    or a real hardware driver), so the same URDF serves real hardware, Gazebo and MVSim.
    Other ``<ros2_control>`` blocks are ignored with a warning. The command interfaces
    in the URDF must match ``<command_interface>`` (and ``position`` for steering
    joints); joints may also be listed with state interfaces only. If the URDF defines
    the sensor frames, set the ``mvsim_node`` parameter ``publish_sensor_tf:=false`` (as
    the demo launch file does), so they are not published twice.

The URDF is only used for control and frames: MVSim physics always come from its own
vehicle XML (wheel positions, diameters, masses, friction), which must be kept consistent
with the URDF: generate it from the URDF with :ref:`mvsim-urdf2xml`, which also sets the
vehicle frame name (``<base_frame_id>``) to that of the URDF frame on the ground, e.g.
``base_footprint`` if ``base_link`` is above it.

|

Examples
----------

``mvsim_tutorial/demo_ros2_control.launch.py`` runs one of these vehicles, each with its
controller and ``joint_state_broadcaster``. The steering controllers are not dependencies
of MVSim: install them to run the Ackermann demos, e.g.
``sudo apt install ros-$ROS_DISTRO-ackermann-steering-controller ros-$ROS_DISTRO-bicycle-steering-controller``.

.. list-table::
   :header-rows: 1

   * - ``vehicle:=``
     - Vehicle class
     - Controller
     - Command topic
   * - ``jackal`` (default)
     - ``differential_4_wheels`` (skid steer, four velocity-controlled wheels)
     - ``diff_drive_controller``
     - ``/diff_drive_controller/cmd_vel``
   * - ``car_ackermann``
     - ``ackermann`` (rear-wheel drive, two steering joints)
     - ``ackermann_steering_controller``
     - ``/ackermann_steering_controller/reference``
   * - ``agricobiot2``
     - ``ackermann_drivetrain`` (open rear differential, central steering joint)
     - ``bicycle_steering_controller``
     - ``/bicycle_steering_controller/reference``
   * - ``skid_steer_4wd``
     - ``differential_4_wheels``, generated from its URDF (``base_footprint`` on the
       ground, ``base_link`` above it) with :ref:`mvsim-urdf2xml`
     - ``diff_drive_controller``
     - ``/diff_drive_controller/cmd_vel``

.. code-block:: bash

    # MVSim-generated robot description:
    ros2 launch mvsim demo_ros2_control.launch.py vehicle:=car_ackermann

    # Or, with an existing URDF whose <ros2_control> block targets Gazebo
    # (mvsim_tutorial/ros2_control/<vehicle>.urdf), published by robot_state_publisher:
    ros2 launch mvsim demo_ros2_control.launch.py vehicle:=car_ackermann robot_description:=urdf

    # Drive it:
    ros2 topic pub /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped \
      "{twist: {linear: {x: 2.0}, angular: {z: 0.3}}}"

Odometry is published by the controller (e.g. ``/diff_drive_controller/odom`` or
``/ackermann_steering_controller/odometry``, and the ``odom -> base_link`` TF), and MVSim
ground truth in ``/base_pose_ground_truth``. The world files are
``mvsim_tutorial/demo_ros2_control*.world.xml``, and the controller parameters and URDF
files are in ``mvsim_tutorial/ros2_control/``.

Skid-steer wheels slip sideways when turning, so the yaw rate from the wheel speeds is
larger than the actual one, as in real robots: calibrate it with the
``wheel_separation_multiplier`` of ``diff_drive_controller`` (as in
``jackal_controllers.yaml`` and ``skid_steer_4wd_controllers.yaml``), or the odometry
drifts in turns.

|

.. _ros2_control_ackermann:

Ackermann vehicles
--------------------

``ackermann`` vehicles have one motor per wheel; ``ackermann_drivetrain`` ones, one motor
whose torque is split by differentials (see :ref:`vehicles`). Both steer the front
wheels. Their joints, with ``<controller class="ros2_control">``:

.. list-table::
   :header-rows: 1

   * - Joint (default name)
     - ``ackermann``
     - ``ackermann_drivetrain``
   * - ``rl_wheel_joint``, ``rr_wheel_joint``, ``fl_wheel_joint``, ``fr_wheel_joint``:
       wheel spin
     - ``velocity`` or ``effort`` command
     - state only (driven by the drivetrain)
   * - ``traction_joint``: the drivetrain, as the traction wheel of a bicycle model. Its
       speed times its radius (that of the driven wheels) is the vehicle speed
     - (none)
     - ``velocity`` or ``effort`` command
   * - ``fl_steer_joint``, ``fr_steer_joint``: steering angle of each front wheel
     - ``position`` command with ``per_wheel`` steering (default), else state only
     - ``position`` command with ``per_wheel`` steering, else state only (default)
   * - ``steer_joint``: equivalent steering angle (bicycle model), at the front axle
       center. MVSim applies it to the wheels with the Ackermann geometry, as a steering
       rack
     - ``position`` command with ``central`` steering
     - ``position`` command with ``central`` steering (default)

All joints have ``position`` and ``velocity`` state interfaces, and traction joints also
``effort``. Positive steering angles turn left.

.. code-block:: xml

    <dynamics class="ackermann">
      <rl_wheel pos="0  1" diameter="0.62" ... />  <!-- joint: "rl_wheel_joint" -->
      ...
      <controller class="ros2_control">
        <steering_joints>per_wheel</steering_joints>  <!-- or "central" -->
        <command_interface>velocity</command_interface>  <!-- traction joints -->
        <KP>2550</KP> <KI>51</KI> <KD>0</KD> <max_torque>287</max_torque>
        <controllers_yaml>ros2_control/car_ackermann_controllers.yaml</controllers_yaml>
        <!-- Optional joint names: -->
        <fl_steer_joint_name>fl_steer_joint</fl_steer_joint_name>
        <fr_steer_joint_name>fr_steer_joint</fr_steer_joint_name>
        <steer_joint_name>steer_joint</steer_joint_name>
        <traction_joint_name>traction_joint</traction_joint_name>
      </controller>
    </dynamics>

- **Which controller:** ``ackermann_steering_controller`` needs two traction and two
  steering joints: an ``ackermann`` vehicle with ``per_wheel`` steering (traction joints:
  any two wheels, e.g. the rear ones). ``bicycle_steering_controller`` needs one traction
  joint and one steering joint: an ``ackermann_drivetrain`` vehicle with ``central``
  steering. ``tricycle_steering_controller`` (two traction joints, one steering joint) fits
  an ``ackermann`` vehicle with ``central`` steering. A bicycle model controller can also
  drive an ``ackermann`` vehicle through chainable controllers that turn its commands
  into per-wheel ones, as on real hardware (e.g.
  `bicycle_to_ackermann_traction_adapter
  <https://github.com/racarla96/caddy_ai2_ros2_bicycle_to_ackermann_traction_adapter>`_ and
  `bicycle_to_ackermann_steering_adapter
  <https://github.com/racarla96/caddy_ai2_ros2_bicycle_to_ackermann_steering_adapter>`_).
- **Driven wheels:** in ``ackermann`` vehicles, the wheels whose joints receive commands;
  the rest roll freely. In ``ackermann_drivetrain`` vehicles, those of the ``<drivetrain>``
  type, and the inner PID acts on the drivetrain speed (as ``twist_front_steer_pid``).
- **Steering:** the wheel angles follow the commands kinematically, limited by
  ``max_steer_rate_deg`` and ``max_steer_ang_deg``. ``max_steer_ang_deg`` limits the
  equivalent angle, so each front wheel (``per_wheel`` steering) is limited to the angle
  it has at that equivalent angle: that of the inner wheel when turning toward its side,
  and that of the outer one otherwise. These limits go to the generated URDF too
  (``lower`` and ``upper``).
  With ``per_wheel`` steering, the angles are applied as given, even if they do not match
  the Ackermann geometry of the vehicle (the tires then slip). A steering joint without a
  command keeps its angle.
- **Geometry:** the controller parameters must match the vehicle XML: ``wheelbase`` is
  ``f_wheels_x`` minus the rear wheels ``x``, the track widths are ``f_wheels_d`` and the
  rear wheels ``y`` distance, and the radii are half the wheel diameters.
- **base_link at the rear axle:** the odometry of the steering controllers is that of the
  center of the rear (traction) axle, so the vehicle origin must be there (rear wheels at
  ``x=0``, as in ``definitions/ackermann.vehicle.xml`` and
  ``definitions/agricobiot2.vehicle.xml``); otherwise, their odometry drifts in turns.
- **URDF:** in the generated one, each front wheel joint hangs from its steering joint
  link (``revolute``, about ``z``, at the wheel center), so ``robot_state_publisher``
  shows the wheels steering. Use the same structure in your URDF (see
  ``mvsim_tutorial/ros2_control/car_ackermann.urdf``). List state-only joints in the
  ``<ros2_control>`` block too, so ``joint_state_broadcaster`` publishes them.
- **ROS 2 distributions:** the steering controllers renamed their parameters in Jazzy
  (``traction_joints_names``, ``steering_joints_names``, ``traction_track_width``, ...);
  Humble only knows the old ones (``rear_wheels_names``, ``front_wheels_names``,
  ``rear_wheel_track``, ...) and newer versions only the new ones. The tutorial YAML files
  set both, so they work in all of them (Jazzy warns about the old ones). They also set
  ``use_stamped_vel: true``, so the reference is a ``TwistStamped`` on ``~/reference``
  in Humble too.

|

Simulation rate vs. controller rate
-------------------------------------

The simulation rate (``1/simul_timestep``) should be an integer multiple of the controller
manager ``update_rate``; otherwise, the closest multiple is used and a warning is printed.
For instance, ``simul_timestep=0.005`` (200 Hz) with ``update_rate: 100`` updates the
controllers every 2 physics steps. Smaller simulation time steps improve the accuracy of the
wheel-ground contact dynamics at a higher CPU cost; 2 to 5 physics steps per controller update
is a good default.
