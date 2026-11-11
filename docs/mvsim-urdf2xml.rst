.. _mvsim-urdf2xml:

mvsim-urdf2xml
==================

``mvsim-urdf2xml`` is a standalone command-line tool that generates an MVSim vehicle class XML
file from a URDF robot description, or checks an existing vehicle XML file against it.

The simulator itself only reads MVSim XML files. This tool lets the URDF remain the single
source of truth for the robot **geometry** (wheel positions and sizes, chassis, sensor poses,
visual meshes), while MVSim-specific parameters (dynamics class, controller, friction, sensor
models) come from a small mapping file. Regenerate the XML whenever the URDF changes, or use
``--check`` (e.g. in CI) to detect when both have diverged.

|

Usage
-------

.. code-block:: bash

    # Expand xacro files first:
    xacro my_robot.urdf.xacro > my_robot.urdf

    mvsim-urdf2xml my_robot.urdf mapping.yaml -o my_robot.vehicle.xml
    mvsim-urdf2xml my_robot.urdf mapping.yaml --check my_robot.vehicle.xml

From ROS 2: ``ros2 run mvsim mvsim-urdf2xml ...``. It only needs Python 3 and PyYAML.

|

Mapping file
--------------

Example (``mvsim_tutorial/urdf2xml/jackal_mapping.yaml``):

.. literalinclude:: ../mvsim_tutorial/urdf2xml/jackal_mapping.yaml
   :language: yaml

- ``base_link``: the URDF link used as the vehicle frame. MVSim vehicle frames are **on the
  ground**: if the URDF ``base_link`` is above the ground, use a frame on the ground (e.g.
  ``base_footprint``). The tool warns if wheel centers are not at their radius height. If
  it is not ``base_link``, its name is written as the vehicle ``<base_frame_id>``, so MVSim
  uses it in ROS (TFs and message ``frame_id``) and does not publish its default
  ``base_link -> base_footprint`` TF: the TF tree is that of the URDF.
- ``dynamics``: ``differential``, ``differential_3_wheels``, ``differential_4_wheels``,
  ``ackermann`` or ``ackermann_drivetrain``; ``wheels`` maps each MVSim wheel tag of that
  class to a URDF joint. Wheel position, diameter, width (from a ``cylinder`` geometry) and
  mass come from the URDF, and the joint name is written as the wheel ``joint_name`` (used
  by :ref:`ros2_control`).
- Ackermann classes: the front axle position and track (``f_wheels_x``, ``f_wheels_d``)
  come from the front wheels. Optionally, ``steering_joints`` lists the URDF steering joints,
  either one central joint or the two front wheel ones, whose ``<limit>`` gives
  ``max_steer_ang_deg`` (for front wheel joints, from the inner wheel limits: the ``upper``
  one of the left wheel and the ``lower`` one of the right wheel) and ``max_steer_rate_deg``
  (from ``velocity``).
- ``chassis``: optional ``link`` whose ``box`` geometry gives the chassis footprint and height,
  and optional overrides ``mass``, ``zmin``, ``zmax``. By default, the mass is the sum of the
  masses of the other links.
- ``controller`` and ``friction``: XML copied verbatim.
- ``sensors``: for each one, the URDF ``link`` (it becomes the sensor name, hence its TF
  ``frame_id``), the MVSim sensor definition file to ``include``, optional extra ``args``
  (values may use variables of the including file, e.g. ``${rate|10}``), and an optional
  ``if`` condition (e.g. ``${use_camera|false}``) that wraps the sensor in an ``<if>`` tag,
  so the world file that includes the vehicle can enable it.
  The sensor pose is computed through the chain of joints from ``base_link``. For pinhole
  cameras (``+Z`` forward), map the URDF optical frame link.
- ``visual`` (default ``true``): emit the mesh visuals of non-wheel links, resolving
  ``package://`` URIs with ``package_paths`` (a map package name to directory) or the ROS
  environment.

|

Examples
----------

- ``mvsim_tutorial/urdf2xml/jackal_mapping.yaml`` (above).
- ``mvsim_tutorial/urdf2xml/skid_steer_4wd_mapping.yaml``: a 4-wheel skid-steer robot whose
  URDF (``mvsim_tutorial/ros2_control/skid_steer_4wd.urdf``) has ``base_footprint`` on the
  ground and ``base_link`` above it. It generates ``skid_steer_4wd.vehicle.xml``, used by
  ``mvsim_tutorial/demo_ros2_control_skid_steer.world.xml``:

  .. code-block:: bash

      ros2 launch mvsim demo_ros2_control.launch.py vehicle:=skid_steer_4wd robot_description:=urdf

  Without ros2_control, the generated vehicle uses its default controller (``twist_pid``,
  driven with ``cmd_vel``) in any world that includes it.

|

Sensor TFs with robot_state_publisher
---------------------------------------

When ``robot_state_publisher`` publishes the URDF TFs, set the ``mvsim_node`` parameter
``publish_sensor_tf:=false`` so the sensor TFs are not published twice.
