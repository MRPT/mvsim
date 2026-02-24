.. _physics:

Physics Models
==============

This document describes the mathematical models and algorithms used in MVSim for simulating
vehicle dynamics, wheel-ground interaction, and friction forces.

.. contents::
   :depth: 1
   :local:
   :backlinks: none


Overview
--------

MVSim uses a modular, hierarchical physics architecture for efficient ground vehicle simulation:

* **Two-Level Physics Engine:**
  
  * **Upper Level** - 2D rigid body dynamics using Box2D for chassis and blocks
  * **Lower Level** - Detailed wheel-ground interaction with realistic friction models

* **Core Components:**

  * **Wheel Dynamics** - Individual wheel rotation and forces
  * **Friction Models** - Tire-ground interaction and slip
  * **Vehicle Dynamics** - Different locomotion systems (differential, Ackermann)
  * **Controllers** - Translating commands to torques

This hierarchical approach focuses on ground vehicles, allowing the use of a simplified 2D
physics engine for body collisions while solving wheel-ground interaction forces separately,
achieving high simulation efficiency without sacrificing realism.

Architecture Overview
---------------------

Software Modules
~~~~~~~~~~~~~~~~

MVSim consists of three main C++ modules with Python bindings:

1. **mvsim-comms:** Client-server communication framework
   
   * Publish-subscribe functionality via ZeroMQ sockets
   * Remote service invocation (RPC)
   * Portable, robust, and efficient data transport

2. **mvsim-msgs:** Message definitions
   
   * Interface Definition Language (IDL) using Google Protobuf
   * Compiled into C++ and Python libraries
   * Standardized data interchange format

3. **mvsim-simulator:** Core simulation engine
   
   * World file parsing and loading
   * Physics simulation (Box2D + custom wheel dynamics)
   * GUI for exploration and manipulation
   * GPU-accelerated sensor simulation

**Integration Options:**

* **Standalone:** ``mvsim-cli`` executable for launching and monitoring simulations
* **Python:** Direct integration via ``mvsim_comms`` and ``mvsim_msgs`` packages
* **ROS 1/2:** Node acting as bridge between MVSim and ROS ecosystem
* **Headless:** Containerized execution for CI/CD pipelines

Physics Engine Selection
~~~~~~~~~~~~~~~~~~~~~~~~~

**Box2D for Rigid Bodies:**

MVSim uses the Box2D physics library :cite:`catto2005iterative` for:

* 2D rigid body dynamics
* Collision detection between vehicles and obstacles
* Efficient constraint solving
* Stable numerical integration

**Why Box2D for Ground Vehicles:**

* Optimized for 2D scenarios (ground vehicle primary motion plane)
* Lightweight and deterministic
* No overhead from full 3D collision detection
* Well-tested and stable in games and simulations

**Custom Wheel Physics:**

Box2D handles chassis dynamics, while MVSim implements custom:

* Realistic tire friction models (Coulomb, Ward-Iagnemma, ellipse curves)
* Wheel slip and rolling resistance
* Torque distribution through differentials
* PID velocity control with simulated odometry feedback

Sensor Simulation
~~~~~~~~~~~~~~~~~

**GPU Acceleration:**

MVSim achieves real-time sensor simulation through OpenGL GPU acceleration:

* **3D LiDAR:** Millions of points per second
* **Depth Cameras:** Real-time range image generation  
* **RGB Cameras:** Hardware-accelerated rendering of 3D scenes
* **Accurate Raytracing:** All depth sensors measure distances to custom 3D models

**Sensor Types:**

* Pin-hole RGB cameras
* RGBD (depth) cameras
* 2D LiDAR scanners (with optional 3D raytracing)
* 3D LiDAR scanners (Velodyne, Ouster, Hesai models)
* IMU (Inertial Measurement Unit)
* GNSS/GPS receivers

**Performance:**

GPU acceleration enables:

* Real-time simulation with multiple high-resolution sensors
* Headless execution in Docker containers with GPU passthrough
* Synthetic dataset generation for SLAM and perception research

1. Wheel Dynamics
-----------------

Kinematics Model
~~~~~~~~~~~~~~~~

Each wheel in MVSim is modeled as a rigid cylinder attached to a vehicle chassis. The velocity
of a wheel center point is determined by rigid body kinematics.

Let :math:`\mathbf{v}_V` be the velocity vector of the vehicle :math:`V` reference point, and
:math:`\boldsymbol{\omega}_V` the vehicle angular velocity. The velocity vector of a wheel
:math:`W` center point, expressed in global coordinates :math:`O`, is given by:

.. math::

   \mathbf{v}_W = \mathbf{v}_V + \boldsymbol{\omega}_V \times {}^V\mathbf{t}_W

where :math:`\times` is the cross product and :math:`{}^V\mathbf{t}_W` is the translational
part of the relative pose :math:`{}^V\mathbf{T}_W` of the wheel with respect to the vehicle
reference frame.

**For planar motion**, this simplifies to:

.. math::

   v_{W,x} = v_{V,x} - \omega_V \cdot y_W

.. math::

   v_{W,y} = v_{V,y} + \omega_V \cdot x_W

where :math:`(x_W, y_W)` is the wheel position relative to the vehicle center, and
:math:`\omega_V` is the vehicle angular velocity around the vertical axis.

Wheel Local Coordinates
~~~~~~~~~~~~~~~~~~~~~~~~

To evaluate tire-ground friction, velocities must be expressed in the wheel's local frame,
where the local x-axis points forward along the wheel's rolling direction.

Let :math:`\theta` be the rotation of the wheel with respect to the vehicle longitudinal axis.
The velocity vector in the wheel frame :math:`W` is:

.. math::

   {}^W\mathbf{v}_W = R_z(\theta)^{-1} \mathbf{v}_W

where :math:`R_z(\theta)` is the rotation matrix around the z-axis:

.. math::

   R_z(\theta) = \begin{bmatrix}
   \cos\theta & -\sin\theta & 0 \\
   \sin\theta & \cos\theta & 0 \\
   0 & 0 & 1
   \end{bmatrix}

Properties
~~~~~~~~~~

A wheel :math:`W` is characterized by:

* Location relative to chassis: :math:`L_w = \{x_w, y_w, \Phi\}` [m, m, rad]
* Diameter: :math:`d_w` [m]
* Width: :math:`w_w` [m]
* Mass: :math:`m_w` [kg]
* Moment of inertia: :math:`I_{yy}` [kg·m²]
* Spinning angular position: :math:`\phi_w` [rad]
* Spinning angular velocity: :math:`\omega_w` [rad/s]

Thus, a wheel is represented as:

.. math::

   W = \{L_w, d_w, w_w, m_w, I_{yy}, \phi_w, \omega_w\}

The moment of inertia for a cylinder is calculated as:

.. math::

   I_{yy} = \frac{1}{2} m_w R^2

where :math:`R = d_w/2` is the wheel radius.

Wheel Forces
~~~~~~~~~~~~

Each wheel experiences the following forces:

.. figure:: imgs/wheel_forces.svg
   :alt: Wheel forces diagram
   :align: center

   Forces acting on a wheel

* **Normal force** :math:`F_z` - Weight on the wheel from chassis [N]
* **Motor torque** :math:`\tau` - Applied by the motor [N·m]
* **Friction forces** :math:`F_x, F_y` - Ground reaction forces [N]
* **Velocity** :math:`\vec{v} = (v_x, v_y)` - Instantaneous velocity in local frame [m/s]

Wheel State Update
~~~~~~~~~~~~~~~~~~

The wheel angular acceleration :math:`\alpha` is computed from torque balance:

.. math::

   \alpha = \frac{\tau_{motor} - R \cdot F_{friction,lon} - C_{damp} \cdot \omega_w − T_{rr}}{I_{yy}}

The angular velocity is updated using explicit Euler integration:

.. math::

   \omega_w(t + \Delta t) = \omega_w(t) + \alpha \cdot \Delta t

And the angular position:

.. math::

   \phi_w(t + \Delta t) = \phi_w(t) + \omega_w(t) \cdot \Delta t

The angular position is wrapped to prevent numerical overflow:

.. math::

   \phi_w \leftarrow \text{fmod}(\phi_w, 2\pi) \quad \text{if } |\phi_w| > 10^4

2. Friction Models
------------------

Friction models determine the forces between wheels and ground based on wheel state,
velocity, and applied torques.

Friction Model Base
~~~~~~~~~~~~~~~~~~~

All friction models implement the ``FrictionBase`` interface, which takes as input:

**Friction Input Structure:**

* Normal force on wheel: :math:`F_z` [N]
* Motor torque: :math:`\tau_{motor}` [N·m]
* Wheel instantaneous velocity in local frame: :math:`\vec{v} = (v_x, v_y)` [m/s]
* Wheel properties: mass, diameter, inertia, current angular velocity
* Simulation timestep: :math:`\Delta t` [s]

**Friction Output:**

* Friction force vector in vehicle frame: :math:`\vec{F}_{friction} = (F_x, F_y)` [N]

Coordinate Frames
~~~~~~~~~~~~~~~~~

Each wheel has its own local coordinate frame:

* **X-axis** - Points forward along wheel rolling direction
* **Y-axis** - Points laterally (perpendicular to rolling direction)
* **Z-axis** - Points upward (normal to ground)

Vehicle velocity must be transformed into each wheel's frame:

.. math::

   \vec{v}_w = R(\Phi_w)^{-1} \cdot \vec{v}_{vehicle}

where :math:`R(\Phi_w)` is the rotation matrix for wheel yaw angle :math:`\Phi_w`.

Default Friction Model
~~~~~~~~~~~~~~~~~~~~~~

The default friction model provides a simple but effective simulation of tire-ground
interaction suitable for indoor robots and general applications.

**XML Configuration:**

.. code-block:: xml

   <friction class="default">
       <mu>0.8</mu>              <!-- Friction coefficient -->
       <C_damping>1.0</C_damping>  <!-- Damping coefficient [N·m·s/rad] -->
       <C_rr>0.01</C_rr>          <!-- Rolling resistance coefficient -->
   </friction>

**Algorithm:**

1. Transform vehicle velocity to wheel frame:

   .. math::

      \vec{v}_w = (v_{wx}, v_{wy}) = R(-\Phi_w) \cdot \vec{v}_{vehicle}

2. Calculate partial mass (portion of vehicle mass on this wheel plus wheel mass):

   .. math::

      m_{wp} = \frac{F_z}{g} + m_w

3. Compute maximum friction force:

   .. math::

      F_{max} = \mu \cdot m_{wp} \cdot g

4. **Lateral friction** (prevents side slip):

   The impulse needed to eliminate lateral velocity:

   .. math::

      F_{y,desired} = -\frac{v_{wy} \cdot m_{wp}}{\Delta t}

   Clamped to maximum friction:

   .. math::

      F_y = \text{clamp}(F_{y,desired}, -F_{max}, F_{max})

5. **Longitudinal friction** (rolling resistance and traction):

   First, compute the desired wheel angular velocity for pure rolling (no slip):

   .. math::

      \omega_{constraint} = \frac{v_{wx}}{R}

   The angular velocity impulse needed:

   .. math::

      \Delta\omega_{desired} = \omega_{constraint} - \omega_w

   Angular acceleration to achieve this:

   .. math::

      \alpha_{desired} = \frac{\Delta\omega_{desired}}{\Delta t}

   From torque balance, the friction force is:

   .. math::

      F_{x,desired} = \frac{\tau_{motor} - I_{yy} \alpha_{desired} - C_{damp} \omega_w - T_{rr}}{R}

   Clamped to maximum friction:

   .. math::

      F_x = \text{clamp}(F_{x,desired}, -F_{max}, F_{max})

6. Recompute actual wheel angular acceleration with limited friction:

   .. math::

      \alpha_{actual} = \frac{\tau_{motor} - R \cdot F_x - C_{damp} \omega_w}{I_{yy}}

7. Update wheel angular velocity:

   .. math::

      \omega_w \leftarrow \omega_w + \alpha_{actual} \cdot \Delta t

8. Transform friction force back to vehicle frame:

   .. math::

      \vec{F}_{vehicle} = R(\Phi_w) \cdot (F_x, F_y)

**Rolling Resistance Torque:**

When ``C_rr`` > 0, a shaft-level torque opposing wheel spin is applied:

.. math::

   T_{rr} = C_{rr} \cdot F_{normal} \cdot R \cdot \tanh(\omega_w \cdot 100)

where :math:`F_{normal}` is the normal force on the wheel, :math:`R` is the wheel radius,
and :math:`\omega_w` is the wheel angular velocity. The :math:`\tanh` provides a smooth
transition near zero angular velocity, avoiding sign discontinuities. This torque is
subtracted from the effective motor torque before computing longitudinal friction.

**Parameters:**

* :math:`\mu` - Coulomb friction coefficient (typical values: 0.7-0.9)
* :math:`C_{damp}` - Viscous damping in wheel bearing [N·m·s/rad]
* :math:`C_{rr}` - Rolling resistance coefficient (dimensionless, typical values: 0.005-0.03). Default: 0 (disabled)

Ward-Iagnemma Friction Model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An enhanced friction model that includes velocity-dependent rolling resistance,
based on the paper by Ward and Iagnemma (2008) :cite:`ward2008dynamic`.

**XML Configuration:**

.. code-block:: xml

   <friction class="wardiagnemma">
       <mu>0.7</mu>
       <C_damping>10</C_damping>
       <A_roll>50</A_roll>    <!-- Rolling resistance shape parameter -->
       <R1>0.0075</R1>        <!-- Static rolling resistance coefficient -->
       <R2>0.02</R2>          <!-- Dynamic rolling resistance coefficient -->
       <C_rr>0.0</C_rr>       <!-- Rolling resistance torque coefficient -->
   </friction>

**Rolling Resistance Model:**

Rolling resistance is modeled as a combination of static and velocity-dependent forces.
The model uses a continuously differentiable formulation:

.. math::

   F_{rr} = -\text{sign}(v_x) \cdot N \cdot \left[R_1 (1 - e^{-A_{roll}|v_x|}) + R_2|v_x|\right]

where:

* :math:`v_x` - Longitudinal wheel velocity [m/s]
* :math:`N` - Normal force on wheel [N]
* :math:`R_1` - Static resistance coefficient (dimensionless)
* :math:`R_2` - Dynamic resistance coefficient [s/m]
* :math:`A_{roll}` - Shape parameter controlling transition from static to dynamic

**Physical Interpretation:**

* At zero velocity: :math:`F_{rr} \approx 0` (smooth, no singularity)
* At low velocities: Static term :math:`R_1` dominates
* At high velocities: Dynamic term :math:`R_2 v_x` dominates
* The exponential provides smooth transition between regimes

**Implementation:**

The rolling resistance force is added to the longitudinal friction calculation:

.. math::

   F_x = \frac{\tau_{motor} - I_{yy} \alpha_{desired} - C_{damp} \omega_w}{R} + F_{rr}

The lateral friction is calculated identically to the default model.

**Parameter Selection:**

Default values from the reference paper:

* :math:`A_{roll} = 50` - Provides smooth transition around 0.1 m/s
* :math:`R_1 = 0.0075` - About 0.75% of normal force
* :math:`R_2 = 0.02` - Velocity-dependent component

These can be tuned for different terrains (asphalt, dirt, sand, etc.).

**Two Rolling Resistance Mechanisms:**

This model supports two independent sources of rolling resistance that may be combined:

1. **Ground-contact force** (:math:`F_{rr}`) from ``R1``, ``R2``, ``A_roll`` — models
   terrain/ground contact drag (velocity-dependent force added to the longitudinal friction
   equation).

2. **Tire deformation torque** (:math:`T_{rr}`) from ``C_rr`` — models tire hysteresis as
   a shaft-level torque opposing wheel spin, using the same formula as the default friction model:

   .. math::

      T_{rr} = C_{rr} \cdot F_{normal} \cdot R \cdot \tanh(\omega_w \cdot 100)

Both may be enabled simultaneously for off-road scenarios where terrain drag and tire
hysteresis are distinct effects. For simpler configurations, use only one: either
``R1``/``R2``/``A_roll`` or ``C_rr``.

Ellipse Curve Friction Method
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A physics-based tire model using elliptical friction curves that accounts for both
slip angle and slip ratio, suitable for vehicle dynamics research and autonomous
driving simulation.

**XML Configuration:**

.. code-block:: xml

   <friction class="ellipse">
       <C_damping>0.05</C_damping>
       <C_rr>0.01</C_rr>          <!-- Rolling resistance coefficient -->

       <!-- Lateral slip parameters -->
       <C_alpha>8.5</C_alpha>
       <slip_angle_saturation>0.1</slip_angle_saturation>  <!-- rad -->
       <C_alpha_s>0.5</C_alpha_s>

       <!-- Longitudinal slip parameters -->
       <C_s>7.5</C_s>
       <slip_ratio_saturation>0.1</slip_ratio_saturation>
       <C_s_alpha>0.5</C_s_alpha>
   </friction>

**Slip Definitions:**

**Slip Angle** :math:`\alpha`:

The angle between the wheel's velocity direction and its heading direction:

.. math::

   \alpha = \arctan2(v_y, v_x) - \delta

where :math:`\delta` is the wheel steering angle. Special handling for reverse motion
ensures :math:`|\alpha| \leq \pi/2`.

**Slip Ratio** :math:`s`:

Measures the difference between wheel surface speed and ground speed:

.. math::

   s = \frac{v_{wheel} - v_x}{|v_{wheel}|}

where :math:`v_{wheel} = \omega_w \cdot R` is the wheel surface speed. Clamped to [-1, 1].

**Saturation Function:**

.. math::

   \text{sat}(x, x_{max}) = \begin{cases}
   x & \text{if } |x| < x_{max} \\
   x_{max} \cdot \text{sign}(x) & \text{otherwise}
   \end{cases}

**Friction Force Equations:**

The ellipse method computes maximum available forces based on the friction ellipse:

**Longitudinal Force:**

.. math::

   F_x^{max} = F_z C_s \text{sat}(s, s_s) \sqrt{1 - C_{s\alpha} \left(\frac{\text{sat}(\alpha, \alpha_s)}{\alpha_s}\right)^2}

**Lateral Force:**

.. math::

   F_y^{max} = F_z C_\alpha \text{sat}(\alpha, \alpha_s) \sqrt{1 - C_{\alpha s} \left(\frac{\text{sat}(s, s_s)}{s_s}\right)^2}

where:

* :math:`F_z` - Normal force [N]
* :math:`C_s` - Longitudinal stiffness coefficient
* :math:`C_\alpha` - Lateral stiffness coefficient
* :math:`s_s` - Slip ratio saturation value
* :math:`\alpha_s` - Slip angle saturation value
* :math:`C_{s\alpha}, C_{\alpha s}` - Coupling coefficients

**Physical Interpretation:**

The square root terms create an elliptical friction envelope: as one force component
increases, the available force in the perpendicular direction decreases. This models
the physical limitation that total tire force cannot exceed a maximum value.

**Implementation Notes:**

1. Compute slip angle and slip ratio from wheel kinematics
2. Calculate maximum available forces from ellipse equations
3. Compute desired forces to achieve target wheel velocities
4. Clamp desired forces to maximum available forces
5. Update wheel angular velocity based on actual applied forces

When ``C_rr`` is set, a rolling resistance torque :math:`T_{rr} = C_{rr} \cdot F_z \cdot R \cdot \tanh(\omega_w \cdot 100)`
is subtracted from the effective motor torque, providing velocity-independent resistance
to wheel spin.

Gravity Slope Handling
~~~~~~~~~~~~~~~~~~~~~~~

All friction models receive per-wheel gravity slope forces via the ``gravSlopeForce``
input field. When a vehicle drives on a terrain elevation map, the component of
gravitational force along the ground plane is computed for each wheel and passed to the
friction model.

The friction model incorporates these forces in both lateral and longitudinal directions:

* **Lateral:** The gravity slope force is added to the impulse that cancels lateral
  velocity, preventing the vehicle from sliding sideways on slopes.
* **Longitudinal:** The gravity slope force is added to the contact-patch friction
  (clamped by remaining friction capacity), counteracting the tendency to roll downhill.

This ensures vehicles remain stationary on slopes when no motor torque is applied,
and behave realistically when driving on uneven terrain with elevation maps.

Per-Region Friction Overrides
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MVSim supports spatially varying friction parameters using ``PropertyRegion`` world elements.
By defining regions with the property names ``friction_mu`` and/or ``friction_C_rr``, you can
override the friction coefficient and rolling resistance on a per-wheel basis depending on
where each wheel contacts the ground.

During each simulation timestep, the world position of every wheel is queried against all
active ``PropertyRegion`` elements. If a wheel falls within a region that defines
``friction_mu`` or ``friction_C_rr``, the region's value overrides the friction model's
default parameter for that wheel only. Wheels outside any friction region use the values
from the vehicle's ``<friction>`` XML configuration as usual.

This enables scenarios such as icy patches, sand zones, or other terrain variations without
modifying vehicle definitions.

**XML Example:**

.. code-block:: xml

   <!-- Ice patch: very low friction coefficient -->
   <element class="property_region">
       <property_name>friction_mu</property_name>
       <value_double>0.15</value_double>
       <x_min>-7</x_min>  <y_min>-2</y_min>  <z_min>-1</z_min>
       <x_max>-3</x_max>  <y_max>2</y_max>   <z_max>2</z_max>
   </element>

   <!-- Sand zone: high rolling resistance -->
   <element class="property_region">
       <property_name>friction_C_rr</property_name>
       <value_double>0.08</value_double>
       <x_min>3</x_min>  <y_min>-2</y_min>  <z_min>-1</z_min>
       <x_max>7</x_max>  <y_max>2</y_max>   <z_max>2</z_max>
   </element>

**Supported properties:**

* ``friction_mu`` (double) — Overrides the Coulomb friction coefficient :math:`\mu`.
  Applies to the Default and Ward-Iagnemma friction models. The Ellipse Curve method
  does not use :math:`\mu` (it uses slip-based curves instead).

* ``friction_C_rr`` (double) — Overrides the rolling resistance coefficient :math:`C_{rr}`.
  Applies to all three friction models.

3. Vehicle Dynamics Models
--------------------------

Vehicle dynamics models define how wheels are arranged and controlled to produce
vehicle motion.

Vehicle Base Class
~~~~~~~~~~~~~~~~~~

The ``VehicleBase`` class provides common functionality:

* **Weight Distribution:** Computes normal force on each wheel

  .. math::

     F_{z,i} = \frac{m_{chassis}}{N_{wheels}} \cdot g

  .. note::
     Current implementation uses uniform weight distribution. Future versions will
     include dynamic weight transfer based on acceleration and chassis center of mass.

* **Wheel Velocity Computation:** Calculates each wheel's velocity from vehicle twist

  For a wheel at position :math:`(x_w, y_w)`:

  .. math::

     \vec{v}_w = \vec{v}_{chassis} + \vec{\omega}_{chassis} \times \vec{r}_w

  In 2D:

  .. math::

     v_{w,x} = v_x - \omega \cdot y_w

  .. math::

     v_{w,y} = v_y + \omega \cdot x_w

* **Force Application:** Applies wheel friction forces to vehicle body through Box2D

Differential Drive Dynamics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two independently driven wheels that control translation and rotation through
differential wheel speeds.

**Variants:**

* ``differential`` - Two wheels (basic configuration)
* ``differential_3_wheels`` - Three wheels (two driven + one caster)
* ``differential_4_wheels`` - Four wheels (two driven pairs)

**Kinematics:**

Vehicle motion is determined by wheel angular velocities :math:`\omega_l, \omega_r`:

**Angular Velocity:**

.. math::

   \omega_{vehicle} = \frac{(\omega_r R_r - \omega_l R_l)}{y_r - y_l}

where :math:`y_l, y_r` are lateral positions of left and right wheels.

**Linear Velocity:**

.. math::

   v_x = \omega_l R_l + \omega \cdot y_l

.. math::

   v_y = 0

.. note::
   The current implementation assumes no lateral slip (:math:`v_y = 0`). This is an
   approximation valid for low-speed indoor robots. Future versions may include
   lateral dynamics.

**Inverse Kinematics (for control):**

Given desired :math:`v_{desired}, \omega_{desired}`, compute wheel velocities:

.. math::

   v_l = v - \frac{\omega \cdot w}{2}

.. math::

   v_r = v + \frac{\omega \cdot w}{2}

where :math:`w = |y_r - y_l|` is the wheel track width.

Ackermann Steering Dynamics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Four-wheeled vehicle with front-wheel steering. Rear wheels provide traction while
front wheels steer.

**Wheel Configuration:**

* Rear wheels: Fixed orientation, independently or jointly driven
* Front wheels: Steered wheels with Ackermann geometry

**Steering Geometry:**

To prevent wheel slip during turns, inner and outer wheels must have different
steering angles. Given desired steering angle :math:`\delta`:

.. math::

   \delta_{outer} = \arctan\left(\frac{1}{\cot(|\delta|) + \frac{w}{2l}}\right)

.. math::

   \delta_{inner} = \arctan\left(\frac{1}{\cot(|\delta|) - \frac{w}{2l}}\right)

where:

* :math:`w` - Track width (distance between front wheels)
* :math:`l` - Wheelbase (distance between front and rear axles)
* Inner/outer determined by turn direction

**Geometric Derivation:**

For a turn with radius :math:`R` measured to the rear axle center:

.. math::

   R = \frac{l}{\tan(\delta)}

The inner wheel turns around radius :math:`R_{inner} = R - w/2` and outer wheel around
:math:`R_{outer} = R + w/2`, leading to the above steering angles.

**Odometry Estimation:**

Similar to differential drive but using rear wheel velocities:

.. math::

   \omega_{vehicle} = \frac{\omega_{rr} R_{rr} - \omega_{rl} R_{rl}}{y_{rr} - y_{rl}}

.. math::

   v_x = \omega_{rl} R_{rl} + \omega \cdot y_{rl}

.. math::

   v_y = 0

Ackermann with Drivetrain
~~~~~~~~~~~~~~~~~~~~~~~~~~

Extended Ackermann model with realistic powertrain including differentials that
distribute torque from a single "engine" to multiple wheels.

**Differential Types:**

**Open Differential:**

Simple torque splitting according to preset ratios:

.. math::

   \tau_{FL} = \tau_{motor} \cdot K_{f} \cdot K_{fL}

.. math::

   \tau_{FR} = \tau_{motor} \cdot K_{f} \cdot (1 - K_{fL})

.. math::

   \tau_{RL} = \tau_{motor} \cdot (1 - K_{f}) \cdot K_{rL}

.. math::

   \tau_{RR} = \tau_{motor} \cdot (1 - K_{f}) \cdot (1 - K_{rL})

where :math:`K_f` is front/rear split and :math:`K_{fL}, K_{rL}` are left/right splits.

**Torsen Limited-Slip Differential:**

Self-locking differential that distributes more torque to the wheel with better
traction, based on the Torsen (torque-sensing) design :cite:`chocholek1988development`.

**Algorithm:**

For two output shafts with angular velocities :math:`\omega_1, \omega_2`:

1. Identify fast and slow wheels:

   .. math::

      \omega_{max} = \max(|\omega_1|, |\omega_2|)

   .. math::

      \omega_{min} = \min(|\omega_1|, |\omega_2|)

2. Compute locking parameter:

   .. math::

      \delta_{lock} = \omega_{max} - b \cdot \omega_{min}

   where :math:`b` is the bias ratio (typically 1.5-3.0)

3. Calculate torque bias:

   .. math::

      \delta_t = \begin{cases}
      \delta_{lock} / \omega_{max} & \text{if } \delta_{lock} > 0 \\
      0 & \text{otherwise}
      \end{cases}

4. Distribute torque:

   If :math:`|\omega_1| > |\omega_2|` (wheel 1 is faster):

   .. math::

      f_1 = K_s (1 - \delta_t), \quad f_2 = (1 - K_s)(1 + \delta_t)

   Otherwise:

   .. math::

      f_1 = K_s (1 + \delta_t), \quad f_2 = (1 - K_s)(1 - \delta_t)

5. Normalize and apply:

   .. math::

      \tau_1 = \frac{f_1}{f_1 + f_2} \tau_{in}, \quad \tau_2 = \frac{f_2}{f_1 + f_2} \tau_{in}

**Physical Meaning:**

* When :math:`\omega_1 \approx \omega_2`: Acts like open differential (50/50 split)
* When one wheel slips: More torque goes to slower (gripping) wheel
* Bias ratio: Maximum torque ratio between wheels (e.g., b=2 means 2:1 max ratio)

**4WD Configuration:**

For all-wheel drive:

1. Center differential splits torque between front and rear (with Torsen if configured)
2. Front differential distributes to left/right front wheels
3. Rear differential distributes to left/right rear wheels

Each differential can be independently configured as open or Torsen type.

4. Motor Controllers
--------------------

Controllers translate high-level motion commands into wheel torques.

Controller Types
~~~~~~~~~~~~~~~~

**Raw Controllers:**

Direct specification of wheel torques and steering angles. No feedback control.

**PID Controllers:**

Closed-loop control using Proportional-Integral-Derivative feedback to track
velocity setpoints.

**Ideal Controllers:**

Perfect tracking (kinematic control). Instantly achieves desired velocities without
dynamics.

Differential Twist PID Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Controls linear velocity :math:`v` and angular velocity :math:`\omega` using PID
regulators on each wheel.

**Control Law:**

1. Compute desired wheel velocities from twist command:

   .. math::

      v_l^{des} = v - \frac{\omega w}{2}, \quad v_r^{des} = v + \frac{\omega w}{2}

2. Compute actual wheel velocities from odometry:

   .. math::

      v_l^{act} = \omega_l R_l, \quad v_r^{act} = \omega_r R_r

3. PID control for each wheel:

   .. math::

      e_i = v_i^{des} - v_i^{act}

   .. math::

      \tau_i = K_P e_i + K_I \int e_i \, dt + K_D \frac{de_i}{dt}

   With integral windup protection:

   .. math::

      \int e_i \, dt = \text{clamp}\left(\int e_i \, dt, -I_{MAX}, I_{MAX}\right)

4. Apply torque limits:

   .. math::

      \tau_i = \text{clamp}(\tau_i, -\tau_{max}, \tau_{max})

**Tuning Guidelines:**

* :math:`K_P` - Proportional gain: Higher values → faster response, may cause oscillation
* :math:`K_I` - Integral gain: Eliminates steady-state error, may cause overshoot
* :math:`K_D` - Derivative gain: Damping, rarely needed for velocity control
* :math:`I_{MAX}` - Prevents integral windup during saturation

Ackermann Twist Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Controls forward velocity and steering angle to achieve desired twist.

**Control Strategy:**

1. Compute turn radius from twist command:

   .. math::

      R = \frac{v}{\omega}

2. Compute steering angle:

   .. math::

      \delta = \arctan\left(\frac{l}{R}\right)

   where :math:`l` is the wheelbase.

3. Apply Ackermann geometry to get individual wheel angles

4. Rotate desired velocity vector by steering angle

5. Use PID to control wheel speeds accounting for different radii in turns

**Special Cases:**

* :math:`\omega \approx 0`: Straight line, :math:`\delta = 0`
* Small :math:`R`: Limit :math:`\delta` to :math:`\delta_{max}`

Ackermann Steering Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Direct control of steering angle and forward speed (alternative to twist control).

**Inputs:**

* Desired steering angle :math:`\delta_{des}`
* Desired forward velocity :math:`v_{des}`

**Implementation:**

Uses the twist controller internally after converting steering + speed to equivalent
twist command.

5. Integration and Timestep
----------------------------

Hierarchical Physics Simulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MVSim employs a two-level hierarchical approach for efficient physics simulation:

**Upper Level (2D Rigid Bodies):**

* Vehicles and obstacles ("blocks") are modeled as 2D rigid bodies using Box2D
* For vehicles with custom 3D meshes, equivalent 2D collision polygons are computed
* Box2D provides collision detection and integrates forces/torques on rigid bodies
* Uses fixed timestep numerical integration

**Lower Level (Wheel-Ground Interaction):**

* Detailed wheel dynamics and friction forces computed separately
* Applied before and after each Box2D physics step
* Accounts for realistic tire-ground interaction

Simulation Loop
~~~~~~~~~~~~~~~

The main simulation loop executes Algorithms 1 and 2 for each timestep :math:`\Delta t`.

**Algorithm 1: Pre-Timestep Processing**

.. code-block:: none

   for all vehicles v:
       # Evaluate chassis weight W on each wheel:
       W[] ← v.weight_on_wheels()
       
       # Evaluate motor torques T (includes kinematics and PID control):
       T[] ← v.invoke_motor_controllers()
       
       for all wheels w[i] in v:
           # Compute wheel center point velocity vector (Eq. 1):
           v_w ← v_g + (-ω * w.y, ω * w.x)
           
           # Transform to wheel local coordinates:
           v_local ← rotate_z(v_w, θ_i)
           
           # Evaluate friction model (see Section 2):
           {F_x, F_y} ← v.friction_model(v_local, T[i], W[i])
           
           # Apply force to vehicle chassis at wheel center:
           v.apply_force({F_x, F_y}, t_W)

**Key Points:**

* ``weight_on_wheels()``: Distributes chassis weight among wheels
* ``invoke_motor_controllers()``: Handles vehicle kinematics (Ackermann vs. differential)
  and PID control to achieve velocity setpoints using simulated odometry
* Wheel slippage affects odometry, realistically replicating real-world behavior
* Forces are applied to chassis via Box2D at wheel contact points

**Box2D Physics Step:**

After Algorithm 1, Box2D integrates rigid body dynamics:

* Updates positions and velocities of all rigid bodies
* Handles collisions between vehicles and obstacles
* Applies computed wheel friction forces

**Algorithm 2: Post-Timestep Processing**

.. code-block:: none

   for all vehicles v:
       # Process all sensors (depth cameras, LiDAR, etc.):
       for all sensors s in v:
           s.process()
       
       # Integrate wheel spinning angles:
       for all wheels w in v:
           w.phi ← wrap_to_pi(w.phi + w.omega * dt)

**Key Points:**

* Sensor simulation uses GPU-accelerated OpenGL pipeline
* Wheel angles are wrapped to prevent numerical overflow
* Ground-truth pose/velocity updated from Box2D state

Terrain Elevation Support
~~~~~~~~~~~~~~~~~~~~~~~~~~

MVSim supports vehicles with 3 or more wheels moving on terrain elevation maps:

1. Determine elevation of each wheel-ground contact point
2. Apply quaternion Horn method :cite:`horn1987closed` to solve for optimal vehicle attitude
3. This allows vehicles to tilt, pitch, and roll on uneven terrain while maintaining
   contact with the ground

**Note:** Despite using a 2D physics engine, MVSim uses SE(3) poses (full 3D position
and orientation) rather than SE(2), enabling certain degrees of freedom for vertical
movement and attitude changes when traversing elevation models.

Timestep Selection
~~~~~~~~~~~~~~~~~~

* **Fixed timestep** (recommended): Set ``<simul_timestep>`` in world file
* **Adaptive timestep**: Set to 0 for automatic determination

Recommended values:

* Indoor robots: 10-20 ms (50-100 Hz)
* Outdoor vehicles: 5-10 ms (100-200 Hz)
* High-speed dynamics: 1-5 ms (200-1000 Hz)

Stability Considerations
~~~~~~~~~~~~~~~~~~~~~~~~

The explicit Euler integration used for wheel dynamics is conditionally stable:

.. math::

   \Delta t < \frac{2}{\omega_n}

where :math:`\omega_n` is the natural frequency of the wheel-ground system.

For most applications, timesteps of 10 ms or less ensure stability.

6. Coordinate Systems and Transformations
-----------------------------------------

MVSim uses SE(3) poses (Special Euclidean group in 3D) to represent vehicle positions
and orientations, even though the underlying physics engine operates in 2D. This enables
vehicles to traverse elevation maps with realistic pitch and roll.

Coordinate Frames
~~~~~~~~~~~~~~~~~

**World Frame (O):**

Fixed inertial reference frame with origin at the simulation world origin.

**Vehicle Frame (V):**

Moving frame attached to vehicle chassis reference point (may or may not coincide with
center of mass), aligned with vehicle heading.

**Wheel Frame (W):**

Rotating frame attached to each wheel, with x-axis along rolling direction, y-axis
along the wheel axle, and z-axis pointing upward.

Homogeneous Transformations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Vehicle pose with respect to world origin is represented as a 4×4 homogeneous
transformation matrix:

.. math::

   {}^O\mathbf{T}_V = \begin{bmatrix}
   {}^O\mathbf{R}_V & {}^O\mathbf{t}_V \\
   \mathbf{0}^T & 1
   \end{bmatrix}

where:

* :math:`{}^O\mathbf{R}_V` is a 3×3 rotation matrix (vehicle orientation in world frame)
* :math:`{}^O\mathbf{t}_V` is a 3×1 translation vector (vehicle position in world frame)

Similarly, wheel pose relative to vehicle:

.. math::

   {}^V\mathbf{T}_W = \begin{bmatrix}
   {}^V\mathbf{R}_W & {}^V\mathbf{t}_W \\
   \mathbf{0}^T & 1
   \end{bmatrix}

Transformation Compositions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Wheel Pose in World Frame:**

.. math::

   {}^O\mathbf{T}_W = {}^O\mathbf{T}_V \cdot {}^V\mathbf{T}_W

**Velocity Transformations:**

For a point rigidly attached to the vehicle, its velocity in the world frame is:

.. math::

   {}^O\mathbf{v}_W = {}^O\mathbf{v}_V + {}^O\boldsymbol{\omega}_V \times {}^V\mathbf{t}_W

where :math:`\times` denotes the cross product.

**Planar Simplifications:**

For planar motion (z = 0, no pitch/roll), rotations reduce to:

.. math::

   R_z(\theta) = \begin{bmatrix}
   \cos\theta & -\sin\theta \\
   \sin\theta & \cos\theta
   \end{bmatrix}

And the cross product simplifies to:

.. math::

   \boldsymbol{\omega} \times \mathbf{r} = \omega \begin{bmatrix} -r_y \\ r_x \end{bmatrix}

**Inverse Transformations:**

To transform from vehicle to wheel coordinates:

.. math::

   {}^W\mathbf{v} = {}^W\mathbf{R}_V \cdot {}^V\mathbf{v} = ({}^V\mathbf{R}_W)^T \cdot {}^V\mathbf{v}

For rotation matrices: :math:`\mathbf{R}^{-1} = \mathbf{R}^T`

SE(3) vs SE(2) Usage
~~~~~~~~~~~~~~~~~~~~

**Why SE(3) for Ground Vehicles:**

Despite using a 2D physics engine, MVSim employs SE(3) (6-DOF) poses instead of
SE(2) (3-DOF) for several reasons:

1. **Terrain Following:** Vehicles can pitch and roll on elevation maps
2. **Realistic Attitude:** Captures vehicle tilt during turns or on slopes  
3. **Sensor Simulation:** 3D LiDAR and cameras require full 3D pose
4. **Multi-Wheel Contact:** Horn quaternion method determines optimal 3D attitude
   from wheel ground contact points

**Constrained Motion:**

While pose is SE(3), motion is primarily constrained to:

* **Translation:** Primarily in x-y plane (z from terrain elevation)
* **Rotation:** Primarily yaw, with pitch/roll from terrain geometry


