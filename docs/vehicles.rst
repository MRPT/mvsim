.. _vehicles:

Vehicles
========

This section describes vehicle definitions, dynamics models, and control systems in MVSim.
Vehicles are mobile robots or platforms that can move through the simulated world using
various locomotion systems.


.. contents::
   :depth: 1
   :local:
   :backlinks: none


1. Overview
------------

Vehicles in MVSim are defined through XML configuration files that specify:

* **Dynamics model** - The type of locomotion (differential drive, Ackermann steering, etc.)
* **Physical properties** - Mass, dimensions, wheel configurations
* **Motor controllers** - How commands translate to wheel torques
* **Friction models** - Ground interaction and traction characteristics
* **Sensors** - Optional sensor attachments (LiDAR, cameras, IMU, etc.)
* **Visual models** - 3D meshes for rendering

2. Vehicle Definition Structure
--------------------------------

Basic XML Structure
~~~~~~~~~~~~~~~~~~~

Vehicles are defined using ``<vehicle:class>`` tags for reusable templates, and ``<vehicle>``
tags for specific instances:

.. code-block:: xml

   <!-- Define a reusable vehicle class -->
   <vehicle:class name="my_robot">
       <dynamics class="differential">
           <!-- Dynamics parameters -->
       </dynamics>
       <friction class="default">
           <!-- Friction parameters -->
       </friction>
       <!-- Additional parameters -->
   </vehicle:class>

   <!-- Create a vehicle instance -->
   <vehicle name="robot1" class="my_robot">
       <init_pose>0 0 0</init_pose>  <!-- x, y, yaw(deg) -->
       <init_vel>0 0 0</init_vel>    <!-- vx, vy, omega(deg/s) -->
   </vehicle>

Common Vehicle Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~

**Chassis Configuration:**

.. code-block:: xml

   <chassis mass="15.0" zmin="0.05" zmax="0.6" color="#0080ff">
       <shape>
           <pt>-0.4 -0.5</pt>
           <pt>-0.4  0.5</pt>
           <pt> 0.4  0.5</pt>
           <pt> 0.4 -0.5</pt>
       </shape>
   </chassis>

**Chassis Attributes:**

* **mass** - Vehicle chassis mass in kilograms
* **zmin**, **zmax** - Minimum and maximum Z coordinates for collision detection
* **color** - RGB color in hex format (e.g., #FF0000)
* **shape** - 2D polygon defining the chassis footprint (list of <pt> elements)
* **shape_from_visual** - Automatically extract collision shape from 3D visual model

**Odometry Configuration:**

.. code-block:: xml

   <odometry
       x_multiplier="1.02"
       y_multiplier="1.01"
       yaw_multiplier="1.03"
   />

Adds realistic noise to odometry by multiplying actual pose increments. Values are typically
drawn from ``$f{1.0 + 0.02*randn()}`` to simulate sensor imperfections.

**Visual Models:**

.. code-block:: xml

   <visual>
       <model_uri>path/to/model.dae</model_uri>
       <model_scale>0.001</model_scale>
       <model_offset_x>0.0</model_offset_x>
       <model_offset_y>0.0</model_offset_y>
       <model_offset_z>0.0</model_offset_z>
       <model_yaw>0.0</model_yaw>
       <model_pitch>0.0</model_pitch>
       <model_roll>90.0</model_roll>
   </visual>

3. Dynamics Models
-------------------

MVSim supports several vehicle dynamics models representing different locomotion systems.

Differential Drive
~~~~~~~~~~~~~~~~~~

**Class:** ``differential``, ``differential_3_wheels``, ``differential_4_wheels``

Two independently driven wheels (or sets of wheels) that control vehicle motion through
differential speeds. Common in mobile robots like TurtleBot.

.. code-block:: xml

   <dynamics class="differential">
       <l_wheel pos="0.0  0.5" mass="4.0" width="0.20" diameter="0.40" />
       <r_wheel pos="0.0 -0.5" mass="4.0" width="0.20" diameter="0.40" />
       
       <chassis mass="15.0" zmin="0.05" zmax="0.6" />

       <controller class="twist_pid">
           <KP>5</KP>
           <KI>10</KI>
           <I_MAX>1</I_MAX>
           <KD>0</KD>
           <max_torque>100</max_torque>
       </controller>
   </dynamics>

**Wheel Parameters:**

* **pos** - Position (x, y) relative to chassis center [meters]
* **mass** - Wheel mass [kg]
* **width** - Wheel width [meters]
* **diameter** - Wheel diameter [meters]

**Kinematics:**

The vehicle angular velocity and linear velocity are computed from wheel speeds:

.. math::

   \omega_{veh} = \frac{\omega_r \cdot R_r - \omega_l \cdot R_l}{y_r - y_l}

.. math::

   v_x = \omega_l \cdot R_l + \omega \cdot y_l

where :math:`\omega_i` is wheel angular velocity, :math:`R_i` is wheel radius,
and :math:`y_i` is the lateral position of wheel :math:`i`.

Ackermann Steering
~~~~~~~~~~~~~~~~~~

**Class:** ``ackermann``

Four-wheeled vehicle with front-wheel steering geometry. The two front wheels turn to
steer the vehicle, while rear wheels remain parallel to the chassis axis.

.. code-block:: xml

   <dynamics class="ackermann">
       <!-- Rear wheels -->
       <rl_wheel pos="0  1" mass="6.0" width="0.30" diameter="0.62" />
       <rr_wheel pos="0 -1" mass="6.0" width="0.30" diameter="0.62" />
       
       <!-- Front wheels -->
       <fl_wheel mass="6.0" width="0.30" diameter="0.62" />
       <fr_wheel mass="6.0" width="0.30" diameter="0.62" />
       
       <f_wheels_x>1.3</f_wheels_x>        <!-- Wheelbase -->
       <f_wheels_d>2.0</f_wheels_d>        <!-- Track width -->
       <max_steer_ang_deg>30.0</max_steer_ang_deg>

       <chassis mass="800.0" zmin="0.15" zmax="1.00" />

       <controller class="twist_front_steer_pid">
           <KP>1500</KP>
           <KI>50</KI>
           <I_MAX>20</I_MAX>
           <KD>0</KD>
           <max_torque>600</max_torque>
       </controller>
   </dynamics>

**Ackermann-Specific Parameters:**

* **f_wheels_x** - Distance from rear to front axle (wheelbase) [meters]
* **f_wheels_d** - Distance between front wheel centers (track width) [meters]
* **max_steer_ang_deg** - Maximum steering angle for each front wheel [degrees]

**Steering Geometry:**

The Ackermann geometry ensures wheels trace correct turning circles:

.. math::

   \alpha_{outer} = \arctan\left(\cot(|\alpha|) + \frac{w}{2l}\right)^{-1}

.. math::

   \alpha_{inner} = \arctan\left(\cot(|\alpha|) - \frac{w}{2l}\right)^{-1}

where :math:`\alpha` is the desired steering angle, :math:`w` is track width,
and :math:`l` is wheelbase.

Ackermann with Drivetrain
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Class:** ``ackermann_drivetrain``

Enhanced Ackermann model with realistic powertrain simulation including differentials.
A single "motor" output is distributed to wheels through differential mechanisms.

.. code-block:: xml

   <dynamics class="ackermann_drivetrain">
       <!-- Same wheel configuration as ackermann -->
       
       <drivetrain type="torsen_4wd">
           <front_rear_split>0.5</front_rear_split>
           <front_rear_bias>1.5</front_rear_bias>
           
           <front_left_right_split>0.5</front_left_right_split>
           <front_left_right_bias>1.5</front_left_right_bias>
           
           <rear_left_right_split>0.5</rear_left_right_split>
           <rear_left_right_bias>1.5</rear_left_right_bias>
       </drivetrain>
   </dynamics>

**Drivetrain Types:**

* ``open_front`` - Front-wheel drive with open differential
* ``open_rear`` - Rear-wheel drive with open differential
* ``open_4wd`` - All-wheel drive with open differentials
* ``torsen_front`` - Front-wheel drive with Torsen limited-slip differential
* ``torsen_rear`` - Rear-wheel drive with Torsen limited-slip differential
* ``torsen_4wd`` - All-wheel drive with Torsen limited-slip differentials

**Differential Parameters:**

* **xxx_split** - Torque distribution ratio (0.5 = 50/50 split)
* **xxx_bias** - Torsen bias ratio (how much more torque can go to wheel with traction)

**Open Differential:**

Torque is split according to the split coefficients:

.. math::

   \tau_{FL} = \tau_{motor} \cdot K_{s,f} \cdot K_{s,frl}

.. math::

   \tau_{FR} = \tau_{motor} \cdot K_{s,f} \cdot (1 - K_{s,frl})

**Torsen Differential:**

Self-locking behavior based on wheel speed differences. The bias ratio :math:`b`
(default 1.5) determines locking strength:

.. math::

   \delta_{lock} = \omega_{max} - b \cdot \omega_{min}

Torque distribution changes dynamically to favor the wheel with more traction.

4. Motor Controllers
----------------------

Controllers translate high-level commands (velocity, steering) into wheel torques.

Raw Controllers
~~~~~~~~~~~~~~~

**Class:** ``raw`` (differential), ``front_steer`` (Ackermann)

Direct control of wheel torques and steering angles. User provides:

* Wheel torques directly
* Steering angle (Ackermann only)

.. code-block:: xml

   <controller class="raw">
       <!-- No parameters needed -->
   </controller>

Twist PID Controllers
~~~~~~~~~~~~~~~~~~~~~

**Class:** ``twist_pid`` (differential), ``twist_front_steer_pid`` (Ackermann)

PID control of linear and angular velocities. Automatically computes required wheel
torques to achieve desired twist commands.

.. code-block:: xml

   <controller class="twist_pid">
       <KP>5</KP>           <!-- Proportional gain -->
       <KI>10</KI>          <!-- Integral gain -->
       <I_MAX>1</I_MAX>     <!-- Integral windup limit -->
       <KD>0</KD>           <!-- Derivative gain -->
       <V>0.0</V>           <!-- Initial linear velocity setpoint -->
       <W>0</W>             <!-- Initial angular velocity setpoint -->
       <max_torque>100</max_torque>
   </controller>

**For Differential Drive:**

Desired wheel velocities are computed from commanded twist :math:`(\nu, \omega)`:

.. math::

   v_l = \nu - \frac{\omega \cdot w}{2}

.. math::

   v_r = \nu + \frac{\omega \cdot w}{2}

where :math:`w` is the lateral distance between wheels.

**For Ackermann:**

Steering angle is computed from turn radius, and PID controls wheel torques
to achieve desired speed while accounting for different wheel speeds in turns.

Ideal Controllers
~~~~~~~~~~~~~~~~~

**Class:** ``twist_ideal``

Perfect velocity control without dynamics. Instantly achieves commanded velocities.
Useful for testing high-level algorithms without worrying about low-level control.

.. code-block:: xml

   <controller class="twist_ideal" />

5. Friction Models
-------------------

Friction models simulate tire-ground interaction, determining forces and wheel slip.

.. note::
   The ``mu`` and ``C_rr`` parameters of any friction model can be overridden spatially
   using ``PropertyRegion`` world elements with the property names ``friction_mu`` and
   ``friction_C_rr``. See :ref:`physics` for details.

Default Friction
~~~~~~~~~~~~~~~~

**Class:** ``default``

Basic friction model with Coulomb friction and viscous damping. Suitable for most
indoor robotics applications.

.. code-block:: xml

   <friction class="default">
       <mu>0.8</mu>              <!-- Friction coefficient -->
       <C_damping>1.0</C_damping>  <!-- Viscous damping [N·m·s/rad] -->
       <C_rr>0.01</C_rr>          <!-- Rolling resistance coefficient -->
   </friction>

**Model Equations:**

Maximum friction force:

.. math::

   F_{max} = \mu \cdot m_{wheel} \cdot g

Lateral friction (prevents side slip):

.. math::

   F_{lat} = \text{clamp}\left(\frac{-v_y \cdot m}{\Delta t}, -F_{max}, F_{max}\right)

Longitudinal friction (accelerates/decelerates wheel):

.. math::

   F_{lon} = \frac{\tau_{motor} - I_{yy} \alpha_{desired} - C_{damp} \omega - T_rr}{R}

Ward-Iagnemma Friction
~~~~~~~~~~~~~~~~~~~~~~

**Class:** ``wardiagnemma``

Advanced model with rolling resistance based on Ward & Iagnemma (2008) :cite:`ward2008dynamic`.
Includes velocity-dependent rolling resistance for more realistic off-road simulation.

.. code-block:: xml

   <friction class="wardiagnemma">
       <mu>0.7</mu>
       <C_damping>10</C_damping>
       <A_roll>50</A_roll>  <!-- Rolling resistance shape parameter -->
       <R1>0.0075</R1>      <!-- Static rolling resistance coefficient -->
       <R2>0.02</R2>        <!-- Dynamic rolling resistance coefficient -->
       <C_rr>0.0</C_rr>     <!-- Rolling resistance torque coefficient -->
   </friction>

**Rolling Resistance:**

.. math::

   F_{rr} = -\text{sign}(v_x) \cdot N \cdot \left(R_1 (1 - e^{-A_{roll}|v_x|}) + R_2|v_x|\right)

This force opposes motion and depends on normal force :math:`N` and velocity :math:`v_x`.

Ellipse Curve Method
~~~~~~~~~~~~~~~~~~~~~

**Class:** ``ellipse``

Physics-based tire model using elliptical friction curves. Models slip angle and
slip ratio effects, suitable for vehicle dynamics research.

.. code-block:: xml

   <friction class="ellipse">
       <C_damping>0.05</C_damping>
       <C_rr>0.01</C_rr>                    <!-- Rolling resistance coefficient -->

       <!-- Lateral slip parameters -->
       <C_alpha>8.5</C_alpha>              <!-- Lateral coefficient -->
       <slip_angle_saturation>0.1</slip_angle_saturation>  <!-- rad -->
       <C_alpha_s>0.5</C_alpha_s>          <!-- Coupling coefficient -->

       <!-- Longitudinal slip parameters -->
       <C_s>7.5</C_s>                      <!-- Longitudinal coefficient -->
       <slip_ratio_saturation>0.1</slip_ratio_saturation>
       <C_s_alpha>0.5</C_s_alpha>          <!-- Coupling coefficient -->
   </friction>

**Slip Angle** :math:`\alpha` - Angle between wheel heading and velocity direction

**Slip Ratio** :math:`s`:

.. math::

   s = \frac{v_{wheel} - v_x}{|v_{wheel}|}

**Friction Forces:**

.. math::

   F_x = F_z C_s \text{sat}(s, s_s) \sqrt{1 - C_{s\alpha} \left(\frac{\text{sat}(\alpha, \alpha_s)}{\alpha_s}\right)^2}

.. math::

   F_y = F_z C_\alpha \text{sat}(\alpha, \alpha_s) \sqrt{1 - C_{\alpha s} \left(\frac{\text{sat}(s, s_s)}{s_s}\right)^2}

6. Predefined Vehicle Classes
-------------------------------

The following predefined vehicle classes are available in the ``definitions/`` directory.

TurtleBot3 Burger
~~~~~~~~~~~~~~~~~

**File:** ``turtlebot3_burger.vehicle.xml``

Small differential-drive educational robot.

**Specifications:**

* Dynamics: Differential drive
* Mass: 1.0 kg (chassis), 0.1 kg (each wheel)
* Wheel diameter: 0.066 m
* Wheel separation: 0.16 m
* Default sensors: RPLidar A2, camera, IMU

Jackal UGV
~~~~~~~~~~

**File:** ``jackal.vehicle.xml``

Four-wheel differential-drive robot for outdoor applications.

**Specifications:**

* Dynamics: Differential (4 wheels)
* Mass: 10.0 kg (chassis)
* Wheel diameter: 0.20 m
* Wheelbase: 0.26 m, Track width: 0.32 m
* Default sensors: 2D LiDAR, 3D LiDAR, camera

Small Robot
~~~~~~~~~~~

**File:** ``small_robot.vehicle.xml``

Generic small differential-drive robot template.

**Specifications:**

* Dynamics: Differential drive
* Mass: 15.0 kg (chassis), 4.0 kg (each wheel)
* Wheel diameter: 0.40 m
* Wheel separation: 1.0 m

Ackermann Car
~~~~~~~~~~~~~

**File:** ``ackermann.vehicle.xml``

Generic car-like vehicle with Ackermann steering.

**Specifications:**

* Dynamics: Ackermann steering
* Mass: 800.0 kg
* Wheel diameter: 0.62 m
* Wheelbase: 1.3 m, Track width: 2.0 m
* Max steering: ±30°
* Drivetrain: Torsen 4WD

Pickup Truck
~~~~~~~~~~~~

**File:** ``pickup.vehicle.xml``

Large Ackermann vehicle representing a pickup truck.

**Specifications:**

* Dynamics: Ackermann steering
* Mass: 2000.0 kg
* Wheel diameter: 0.8 m
* Wheelbase: 3.5 m
* Max steering: ±40°

AgriCobiot2
~~~~~~~~~~~

**File:** ``agricobiot2.vehicle.xml``

Agricultural robot with custom 3D model and Ackermann drivetrain.

**Specifications:**

* Dynamics: Ackermann with drivetrain
* Mass: 97.0 kg
* Wheel diameter: 0.406 m
* Wheelbase: 0.62 m
* Open rear-wheel drive

7. Creating Custom Vehicles
----------------------------

Step 1: Define Vehicle Class
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a new XML file in the ``definitions/`` directory:

.. code-block:: xml

   <vehicle:class name="my_custom_robot">
       <dynamics class="differential">
           <!-- Define wheels -->
           <l_wheel pos="0.0  0.3" mass="2.0" width="0.10" diameter="0.20" />
           <r_wheel pos="0.0 -0.3" mass="2.0" width="0.10" diameter="0.20" />
           
           <!-- Define chassis -->
           <chassis mass="20.0" zmin="0.0" zmax="0.5">
               <shape>
                   <pt>-0.3 -0.3</pt>
                   <pt>-0.3  0.3</pt>
                   <pt> 0.3  0.3</pt>
                   <pt> 0.3 -0.3</pt>
               </shape>
           </chassis>
           
           <!-- Choose controller -->
           <controller class="twist_pid">
               <KP>10</KP>
               <KI>5</KI>
               <I_MAX>2</I_MAX>
               <max_torque>50</max_torque>
           </controller>
       </dynamics>
       
       <!-- Choose friction model -->
       <friction class="default">
           <mu>0.8</mu>
           <C_damping>0.5</C_damping>
       </friction>
   </vehicle:class>

Step 2: Instantiate in World File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <mvsim_world>
       <!-- Include your vehicle definition -->
       <include file="definitions/my_custom_robot.vehicle.xml" />
       
       <!-- Create instances -->
       <vehicle name="robot1" class="my_custom_robot">
           <init_pose>0 0 0</init_pose>
       </vehicle>
   </mvsim_world>

Step 3: Add Sensors (Optional)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <vehicle name="robot1" class="my_custom_robot">
       <init_pose>0 0 0</init_pose>
       
       <!-- Add LiDAR -->
       <include file="definitions/lidar2d.sensor.xml"
           sensor_x="0.2" sensor_z="0.3"
           sensor_name="lidar1"
       />
       
       <!-- Add camera -->
       <include file="definitions/camera.sensor.xml"
           sensor_x="0.2" sensor_z="0.25"
           sensor_name="cam1"
       />
   </vehicle>

8. Advanced Features
---------------------

Linked Visual Objects
~~~~~~~~~~~~~~~~~~~~~

For Ackermann vehicles with visible steering mechanisms:

.. code-block:: xml

   <fl_wheel mass="6.0" width="0.30" diameter="0.62"
             linked_yaw="support-left-front-wheel"
             linked_yaw_offset_deg="-90">
       <!-- This wheel's yaw angle controls a named visual object -->
   </fl_wheel>

The ``linked_yaw`` attribute names a visual object whose yaw angle will track the
wheel's steering angle, offset by ``linked_yaw_offset_deg``.

Shape from Visual Model
~~~~~~~~~~~~~~~~~~~~~~~

Automatically extract collision shape from 3D mesh:

.. code-block:: xml

   <chassis mass="10.0" zmin="0.0" zmax="0.5">
       <shape_from_visual />
   </chassis>

This computes the chassis polygon and height from the bounding box of visual models.


Multiple Visual Models
~~~~~~~~~~~~~~~~~~~~~~

Add multiple 3D models to a vehicle:

.. code-block:: xml

   <visual>
       <name>base</name>
       <model_uri>models/base.dae</model_uri>
   </visual>
   <visual>
       <name>sensor_mast</name>
       <model_uri>models/mast.dae</model_uri>
       <model_offset_z>0.5</model_offset_z>
   </visual>

9. Debugging and Logging
-------------------------

Enable vehicle logging for debugging:

.. code-block:: xml

   <log_path>/path/to/logs/</log_path>

This creates CSV files:

* ``mvsim_<name>_pose.csv`` - Vehicle pose and velocity over time
* ``mvsim_<name>_wheel_<N>.csv`` - Per-wheel forces, torques, and velocities

10. ROS 2 Topics
-----------------

MVSim publishes the following ROS 2 topics for each vehicle. Topic names are prefixed with the vehicle name when multiple vehicles exist in the simulation, or published directly when only one vehicle is present.

Standard Vehicle Topics
~~~~~~~~~~~~~~~~~~~~~~~

Always published for each vehicle:

* ``<VEH>/odom`` (nav_msgs/Odometry) - Odometry computed from wheel encoders with configurable noise
* ``<VEH>/base_pose_ground_truth`` (nav_msgs/Odometry) - Perfect ground truth pose and velocity
* ``<VEH>/collision`` (std_msgs/Bool) - Collision detection flag, true when vehicle collides
* ``<VEH>/chassis_markers`` (visualization_msgs/MarkerArray) - RViz markers for chassis and wheel visualization
* ``<VEH>/chassis_polygon`` (geometry_msgs/Polygon) - 2D footprint polygon of the vehicle
* ``<VEH>/tf`` (tf2_msgs/TFMessage) - Dynamic transforms (odom→base_link, sensor frames)
* ``<VEH>/tf_static`` (tf2_msgs/TFMessage) - Static transforms (base_link→base_footprint)

Fake Localization Topics
~~~~~~~~~~~~~~~~~~~~~~~~~

Published when ``do_fake_localization`` parameter is enabled:

* ``<VEH>/amcl_pose`` (geometry_msgs/PoseWithCovarianceStamped) - Fake AMCL localization output
* ``<VEH>/particlecloud`` (geometry_msgs/PoseArray) - Single-particle fake particle filter

Sensor Topics
~~~~~~~~~~~~~

Published dynamically based on sensors attached to the vehicle:

**2D LiDAR:**

* ``<VEH>/<SENSOR_LABEL>`` (sensor_msgs/LaserScan) - 2D laser scan data

**3D LiDAR:**

* ``<VEH>/<SENSOR_LABEL>_points`` (sensor_msgs/PointCloud2) - 3D point cloud (XYZ, XYZRGB, XYZIRT)

**Camera:**

* ``<VEH>/<SENSOR_LABEL>/image_raw`` (sensor_msgs/Image) - Camera image
* ``<VEH>/<SENSOR_LABEL>/camera_info`` (sensor_msgs/CameraInfo) - Camera calibration parameters

**Depth Camera (RGBD):**

* ``<VEH>/<SENSOR_LABEL>_image`` (sensor_msgs/Image) - RGB image
* ``<VEH>/<SENSOR_LABEL>_image_camera_info`` (sensor_msgs/CameraInfo) - RGB camera calibration
* ``<VEH>/<SENSOR_LABEL>_depth`` (sensor_msgs/Image) - Depth image (16UC1, values in millimeters)
* ``<VEH>/<SENSOR_LABEL>_depth_camera_info`` (sensor_msgs/CameraInfo) - Depth camera calibration
* ``<VEH>/<SENSOR_LABEL>_points`` (sensor_msgs/PointCloud2) - Point cloud (XYZ or XYZRGB)

**IMU:**

* ``<VEH>/<SENSOR_LABEL>`` (sensor_msgs/Imu) - Inertial measurement data

**GNSS/GPS:**

* ``<VEH>/<SENSOR_LABEL>`` (sensor_msgs/NavSatFix) - GPS position with optional covariance

Coordinate Frames
~~~~~~~~~~~~~~~~~

MVSim uses the REP-105 standard coordinate frames:

* ``map`` - World fixed frame (only with fake_localization)
* ``odom`` - Odometry frame, drift-free over short periods
* ``base_link`` - Robot body frame at the center of rotation
* ``base_footprint`` - Projection of base_link onto the ground plane
* ``<sensor_name>`` - Individual sensor frames (camera, lidar, etc.)

CSV Logger → ROS 2 publishing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If publishing log topics is enabled via ``publish_log_topics:=True``:


.. code-block:: bash

   ros2 launch mvsim launch_world.launch.py \
      world_file:=my.world.xml \
      publish_log_topics:=True

these topics will be created (per vehicle, lazily on first data):


* ``<VEH>/log/pose/Timestamp``          (Float64)
* ``<VEH>/log/pose/q0x``                (Float64)
* ``<VEH>/log/pose/q1y``                (Float64)
* ``<VEH>/log/pose/dqx``                (Float64)

...

* ``<VEH>/log/wheel_1/torque``          (Float64)
* ``<VEH>/log/wheel_1/friction_x``      (Float64)

...



11. ROS 2 Subscribed Topics
----------------------------

Command Topics
~~~~~~~~~~~~~~

Each vehicle subscribes to:

* ``<VEH>/cmd_vel`` (geometry_msgs/Twist) - Velocity commands (linear.x, linear.y, angular.z)

The command is processed by the vehicle's controller (twist_pid, twist_ideal, etc.) and converted to wheel torques. Commands older than 1 second are automatically discarded for safety.