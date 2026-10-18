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
           <KP>64</KP>
           <KI>10</KI>
           <KD>0</KD>
           <max_torque>14.44</max_torque>
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
           <KP>4000</KP>
           <KI>50</KI>
           <KD>0</KD>
           <max_torque>400</max_torque>
       </controller>
   </dynamics>

**Ackermann-Specific Parameters:**

* **f_wheels_x** - Distance from rear to front axle (wheelbase) [meters]
* **f_wheels_d** - Distance between front wheel centers (track width) [meters]
* **max_steer_ang_deg** - Maximum steering angle for each front wheel [degrees]
* **max_steer_rate_deg** - Maximum steering rate [degrees/s] of the equivalent steering
  angle (optional, also for ``ackermann_drivetrain``). Default: 0, instantaneous steering.

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

Torque-sensing differential: both outputs turn together (locked) until the ratio of
their torques reaches the bias ratio :math:`b` (``xxx_bias``, default 1.5), and then they
slip. So, when one wheel loses traction (e.g. on ice), the other one can still get up to
:math:`b` times its torque, instead of the same torque as with an open differential. See
:ref:`physics` for the equations.

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
       <KP>64</KP>          <!-- Proportional gain [N.m/(m/s)] -->
       <KI>10</KI>          <!-- Integral gain -->
       <KD>0</KD>           <!-- Derivative gain -->
       <enable_antiwindup>false</enable_antiwindup>  <!-- Optional -->
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

**Stopping:** with a zero setpoint, the controller brakes with its proportional action
only (the integral term is reset), so the vehicle comes to a full stop without coasting
or rebounding.

**Skid-steer robots** (``differential_4_wheels``) turn slower than their wheel speeds imply,
since their wheels must slide sideways to turn (e.g. ~60% of the commanded rate for the
Jackal, spinning in place). As in ROS ``diff_drive_controller``, the optional
``<wheel_separation_multiplier>`` (default 1) multiplies the wheel separation used to
compute the wheel speed setpoints to compensate it.

**Tuning:** keep ``max_torque`` below the traction limit of the driven wheels
(:math:`\mu \, N \, R`, with :math:`N` the wheel load), except for skid-steer vehicles,
which need wheel slip to turn. The gains of the predefined vehicle classes were tuned with
:ref:`mvsim-pid-tuner` for fast, overshoot-free tracking with time steps from 1 to 5 ms, and
on low-friction ground.

**For Ackermann:**

Steering angle is computed from turn radius, and PID controls the front (driven) wheel
torques to achieve the desired speed of each wheel, measured from its own spin.

Ideal Controllers
~~~~~~~~~~~~~~~~~

**Class:** ``twist_ideal``

Perfect velocity control without dynamics. Instantly achieves commanded velocities.
Useful for testing high-level algorithms without worrying about low-level control.
Available for ``differential`` and ``ackermann`` vehicles. Note that it imposes the
twist at the vehicle frame origin, even if it is not kinematically feasible (e.g. an
Ackermann vehicle whose origin is not on the rear axle).

.. code-block:: xml

   <controller class="twist_ideal" />

.. _exactly-reproducible-trajectories:

Exactly Reproducible Trajectories
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Class:** ``trajectory``

Drives the vehicle along an EXACT, closed-form ``(t, x, y)`` polyline given directly in
the XML, instead of being teleoperated or controlled by a planner. This is useful for
offline experiments that need repetitive or predefined motion: benchmarking a
perception/SLAM pipeline against the very same path run after run, regression-testing
sensor data, or reproducing a reference trajectory from a dataset.

Like ``twist_ideal``, the controller works for any non-holonomic vehicle type
(``differential``, ``differential_3_wheels``, ``differential_4_wheels`` and ``ackermann``)
and imposes the resulting twist directly on the vehicle state, bypassing the physical
wheel torque model.

Internally, ``mvsim::PoseTrajectoryFollower`` implements the algorithm and has no
dependency on the rest of the simulator, so it can be tested standalone (see
``tests/test_pose_trajectory_follower.cpp``). At each simulation step it:

1. Evaluates the waypoint list at the current simulation time to get the *reference*
   position and a feedforward *speed* (computed automatically as the distance between
   consecutive waypoints divided by their time difference -- so you never specify speed
   directly, only where the vehicle should be and when).
2. Looks ahead ``lookahead_distance`` further along the path geometry from that reference
   position, and computes the local-frame ``(vx, omega)`` twist that steers the vehicle
   towards that lookahead point using the classic pure-pursuit curvature formula
   :math:`\kappa = 2y/L^2`.
3. Optionally clamps ``|omega|`` to ``max_angular_speed``, slowing ``vx`` down (instead of
   just capping the turn rate) so the *curvature* actually driven matches what pure-pursuit
   asked for -- this is what makes the vehicle round off sharp polyline corners realistically
   rather than either teleporting or cutting them short.

Because the pure-pursuit lookahead search is anchored on the time schedule (not on the
vehicle's actual position), tracking remains stable even for self-intersecting paths
(e.g. a figure-eight) and even if the vehicle temporarily lags behind schedule.

.. code-block:: xml

   <controller class="trajectory" loop="true">
       <lookahead_distance>0.6</lookahead_distance>  <!-- [m], default 0.5 -->
       <max_angular_speed>1.5</max_angular_speed>    <!-- [rad/s], default 2.0. 0 = unlimited -->
       <viz_height>0.5</viz_height>                  <!-- [m], default 0.5 -->

       <!-- At least 2 waypoints are required. Times must be strictly increasing,
            but need not start at 0 nor be given in order. -->
       <waypoint t="0.0"  x="0.0" y="0.0" />
       <waypoint t="4.0"  x="4.0" y="0.0" />
       <waypoint t="8.0"  x="4.0" y="4.0" />
       <waypoint t="12.0" x="0.0" y="4.0" />
       <waypoint t="16.0" x="0.0" y="0.0" />
   </controller>

**Parameters:**

* **loop** (attribute, ``<controller>`` tag) - ``true`` (default) to repeat the trajectory
  forever, wrapping the simulation time back to the first waypoint's time each period.
  ``false`` to run it once and then stop (hold the last waypoint's position) -- a "play once"
  trajectory.
* **lookahead_distance** - Pure-pursuit lookahead distance [m] (default 0.5). Larger values
  round off sharp corners more gently but track the geometric path less tightly.
* **max_angular_speed** - Maximum ``|omega|`` [rad/s] the controller will ever command
  (default 2.0). Set to 0 to disable the clamp.
* **waypoint** (one or more) - ``t`` [s], ``x``/``y`` [m] in global coordinates. A segment
  with two waypoints at the same ``(x,y)`` makes the vehicle pause there for the elapsed
  ``t`` difference.
* **viz_height** - Height [m] above ground (default 0.5) at which the trajectory polyline
  is drawn in the 3D view (see below). Purely cosmetic, does not affect tracking.

The full path polyline can be shown in the GUI as a 3D line (a ``mrpt::opengl`` line set),
toggled with the **"Trajectories"** item of the GUI "View" menu. Whether it starts shown or hidden is controlled per-world via the ``<gui>`` block:

.. code-block:: xml

   <gui>
       <show_trajectories>true</show_trajectories>  <!-- default: false -->
   </gui>

.. tip::
   Since trajectories are plain XML, a set of predefined ones can be kept as separate
   files (e.g. under ``definitions/trajectories/``) and swapped in via ``<include>`` and a
   ``<variable>``, without touching the rest of the world file. See the
   ``demo_trajectory.world.xml`` and ``demo_trajectory_ackermann.world.xml`` demos in
   :doc:`demo_worlds` for a complete example (square loop, figure-eight, and
   point-to-point presets, selectable from a single ``TRAJECTORY`` variable).

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
(computed in the wheel frame)

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
* Max steering: ±30°, at up to 60°/s
* Front-wheel drive (its ``<drivetrain>`` block, Torsen 4WD, only applies if the dynamics
  class is changed to ``ackermann_drivetrain``)
* Controller: ``twist_front_steer_pid`` by default; select another one (e.g.
  ``twist_ideal``) with the ``controller`` include variable:
  ``<include file="ackermann.vehicle.xml" controller="twist_ideal" />``

Pickup Truck
~~~~~~~~~~~~

**File:** ``pickup.vehicle.xml``

Large Ackermann vehicle representing a pickup truck.

**Specifications:**

* Dynamics: Ackermann steering
* Mass: 2000.0 kg
* Wheel diameter: 0.8 m
* Wheelbase: 3.5 m
* Max steering: ±40°, at up to 45°/s

AgriCobiot2
~~~~~~~~~~~

**File:** ``agricobiot2.vehicle.xml``

Agricultural robot with custom 3D model and Ackermann drivetrain.

**Specifications:**

* Dynamics: Ackermann with drivetrain
* Mass: 97.0 kg
* Wheel diameter: 0.406 m
* Wheelbase: 0.62 m
* Max steering: ±35°, at up to 90°/s
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
wheel's steering angle, offset by ``linked_yaw_offset_deg``. Since the collision shape
from ``<shape_from_visual/>`` uses the pose of each visual given in the XML, set the
``model_yaw`` of that visual to its yaw with straight wheels (``linked_yaw_offset_deg``).

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

.. _vehicles-suspension:

Chassis Suspension
~~~~~~~~~~~~~~~~~~

Vehicles can optionally simulate a suspension: a spring and a damper at each wheel,
between the chassis and the wheels. The chassis then pitches when accelerating and
braking, rolls in turns and bounces over bumps, and so do the sensors and visual models
mounted on it, while the wheels stay on the ground. The suspension also sets the wheel
loads (dynamic load transfer). It is disabled by default. See
:ref:`the physics model <physics-suspension>`.

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_suspension.mp4" type="video/mp4">
     </video>
   </div>

See also :ref:`videos comparing two damping ratios <physics-suspension-damping>`.

.. code-block:: xml

   <vehicle:class name="my_car">
       <dynamics class="ackermann"> ... </dynamics>

       <suspension enabled="true">
           <frequency_hz>1.3</frequency_hz>     <!-- Heave natural frequency -->
           <damping_ratio>0.3</damping_ratio>
           <!-- Optional: -->
           <wheel index="0" stiffness="35000" damping="2500" />  <!-- per-wheel override -->
           <I_xx>450</I_xx>  <I_yy>1800</I_yy>  <!-- chassis inertia about its COM [kg.m^2] -->
           <cog_height>0.55</cog_height>         <!-- chassis COM height [m] -->
       </suspension>
   </vehicle:class>

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Parameter
     - Default
     - Description
   * - ``enabled`` (attribute)
     - false
     - Enables the suspension.
   * - ``frequency_hz``
     - 1.5
     - Heave natural frequency [Hz]. Each wheel gets the spring stiffness
       :math:`k_i = m_i (2 \pi f)^2` for its share :math:`m_i` of the static chassis weight.
       Typical values: 1-1.5 Hz (cars), 1.5-2.5 Hz (trucks, sports cars).
   * - ``damping_ratio``
     - 0.3
     - Damping ratio of the heave motion: :math:`c_i = 2 \zeta (2 \pi f) m_i`.
   * - ``<wheel index="i" stiffness="k" damping="c"/>``
     - (from above)
     - Spring stiffness [N/m] and damping [N.s/m] of wheel ``i`` (in the order of the
       dynamics class, e.g. 0=rear left for Ackermann vehicles).
   * - ``I_xx``, ``I_yy``
     - box
     - Chassis inertia about its center of mass, around the longitudinal and lateral axes
       [kg.m^2]. Default: that of a box with the chassis footprint and height.
   * - ``cog_height``
     - (zmin+zmax)/2
     - Height of the chassis center of mass over the ground [m].

When the vehicle is loaded, the vibration modes of the chassis are logged, e.g.
``Suspension modes: roll 0.96 Hz (zeta=0.24), pitch 1.24 Hz (zeta=0.28), heave 1.40 Hz
(zeta=0.30)``. Vehicles with a narrow track and a high center of mass need stiff springs:
if they are too soft, the chassis would tip over, which is reported as an error.

The predefined Ackermann car, pickup and Scania truck have a suspension block, disabled
unless the variable ``suspension`` is set to ``true`` where they are included:

.. code-block:: xml

   <include file="../definitions/pickup.vehicle.xml" suspension="true" />

Notes:

* The vehicle pose (e.g. ``base_link`` in ROS) is that of the body with the wheels, so the
  chassis motion appears in the sensor data, while the sensor poses on the vehicle stay
  constant, as in a real vehicle. Use ``Simulable::getChassisPose()`` for the chassis
  pose. IMUs measure the chassis roll and pitch rates and accelerations.
* Custom visual models are mounted on the chassis. If a model includes the wheels in the
  same mesh, they will move with the chassis: define the wheels as separate visual models
  instead (as in the Scania truck).
* Not supported for vehicles that cannot stand on their wheels alone, e.g. semi-trailers.

9. Debugging and Logging
-------------------------

Enable vehicle logging for debugging:

.. code-block:: xml

   <log_path>/path/to/logs/</log_path>

This creates CSV files:

* ``mvsim_<name>_pose.csv`` - Vehicle pose and velocity over time (and the chassis
  suspension displacement ``susp_z``, ``susp_roll``, ``susp_pitch``, if enabled)
* ``mvsim_<name>_wheel_<N>.csv`` - Per-wheel forces, torques, and velocities

10. ROS 2 Topics
-----------------

MVSim publishes the following ROS 2 topics for each vehicle. Topic names are prefixed with the vehicle name when multiple vehicles exist in the simulation, or published directly when only one vehicle is present.

.. note::
   **Simulation time.** MVSim is the ROS *time source*: it publishes the global
   ``/clock`` topic and stamps every outgoing message header with **simulation
   time** (the wall-clock time at simulation start plus the elapsed simulated
   seconds). This keeps all stamps coherent regardless of the real-time factor
   or transient CPU load. Run your downstream nodes with ``use_sim_time:=true``
   so they consume ``/clock``; the ``mvsim_node`` itself drives the clock and
   normally runs with ``use_sim_time:=false``. Set the ``disable_sim_time_clock``
   parameter to ``true`` to opt out and revert to plain wall-clock stamps (no
   ``/clock`` publication), as done before simulation time support was added.

Global Topics
~~~~~~~~~~~~~

* ``/clock`` (rosgraph_msgs/Clock) - Simulation time, for nodes running with ``use_sim_time:=true``

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
* TF ``map`` → ``odom``, such that ``map`` → ``base_link`` is the ground truth pose (even with noisy odometry, or a vehicle not starting at the origin)

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