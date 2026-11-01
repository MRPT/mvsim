.. _mvsim-pid-tuner:

mvsim-pid-tuner
===============

``mvsim-pid-tuner`` is a command-line tool that tunes the twist PID controller of an MVSim
vehicle definition, by simulating the vehicle with many candidate gains and keeping the ones
that track twist commands best.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

Overview
--------

Each candidate set of gains drives the vehicle, in an empty world, through these twist
command profiles:

* **stop_go**: forward, stop, forward, stop, backward, stop.
* **s_curve**: straight, left turn, right turn, left turn, straight, stop.
* **fwd_back**: forward, then backward, then stop.
* **spin** (differential robots only): turn in place, then stop.

The candidates are evaluated at each requested simulation time step (by default 1 ms and
5 ms, the latter being the automatic time step of most worlds), and also on a low-friction
ground (by default :math:`\mu=0.3`, through a ``friction_mu`` property region covering the
world), so the gains are robust to both.

The cost of a candidate is the velocity tracking error: the integral of the absolute error of
the actual chassis linear and angular velocities (not the wheel odometry), the latter
weighted by half the wheel track (differential) or wheelbase (Ackermann). It is increased if
the linear velocity overshoots more than 5%, if the vehicle does not come to a full stop, and
candidates that become unstable are discarded.

The search starts with a coarse grid: ``KP`` values in powers of 2 around an estimate from
the vehicle mass and wheel radius, times several ``KI/KP`` ratios (and several
``wheel_separation_multiplier`` values for skid-steer robots). It is then refined around the
best candidate. Among the candidates within 2% of the best cost, the one with the smallest
``KP`` is selected: tracking is often limited by ``max_torque``, so larger gains would only
make the controller stiffer.

Candidates are evaluated in parallel threads.

Usage
-----

.. code-block:: bash

   mvsim-pid-tuner <vehicle.xml> <vehicle_class> [options]

**Positional arguments:**

* ``vehicle.xml`` -- Path to the vehicle definition XML file
  (e.g., ``definitions/small_robot.vehicle.xml``)
* ``vehicle_class`` -- Vehicle class name as defined in the XML
  (e.g., ``small_robot``)

**Options:**

.. list-table::
   :header-rows: 1
   :widths: 22 15 63

   * - Flag
     - Default
     - Description
   * - ``-v``, ``--speed``
     - 1.0
     - Linear speed of the test profiles [m/s]. Use a typical speed of the vehicle.
   * - ``-r``, ``--radius``
     - auto
     - Turn radius of the S-curve profile [m]. Default: 2 times the minimum turn radius
       (Ackermann) or 1.5 m (differential).
   * - ``--spin-rate``
     - 1.0
     - Angular speed of the spin profile [rad/s] (differential robots).
   * - ``-s``, ``--sim-steps``
     - 0.001,0.005
     - Comma-separated simulation time steps to evaluate [s].
   * - ``--low-mu``
     - 0.3
     - Friction coefficient of the extra low-friction evaluation (0 to disable it).
   * - ``--max-torque``
     - auto
     - Fixed ``max_torque`` [N.m]. Default: the one of the twist PID controller in the XML, or
       90% of the traction limit of the driven wheels if there is none (3 times it for
       skid-steer robots, which need their wheels to slip to turn).
   * - ``-j``, ``--threads``
     - all cores
     - Number of parallel threads.

Example
-------

.. code-block:: bash

   mvsim-pid-tuner definitions/agricobiot2.vehicle.xml agricobiot2 -v 0.5

Sample output:

.. code-block:: text

   Vehicle 'agricobiot2' from 'definitions/agricobiot2.vehicle.xml'
     Dynamics: ackermann_drivetrain, 4 wheels. Traction limit per driven wheel: 50.11 N.m
     Profiles: speed=0.5 m/s, turn radius=1.77 m. max_torque=100 N.m

   Coarse search (56 candidates):
     Evaluated 56/56
   Refinement (15 candidates):
     Evaluated 15/15

   Best candidates:
               KP=2719      KI=192.3     max_torque=100      | cost=2.791    tracking=2.791    overshoot= 0.0%
               KP=2719      KI=271.9     max_torque=100      | cost=2.792    tracking=2.792    overshoot= 0.2%
               KP=1923      KI=271.9     max_torque=100      | cost=2.806    tracking=2.806    overshoot= 0.3%
               KP=2719      KI=384.6     max_torque=100      | cost=2.807    tracking=2.807    overshoot= 0.5%
               KP=1923      KI=192.3     max_torque=100      | cost=2.807    tracking=2.807    overshoot= 0.1%
     SELECTED  KP=1360      KI=271.9     max_torque=100      | cost=2.818    tracking=2.818    overshoot= 0.5%
     XML       KP=800       KI=100       max_torque=100      | cost=3.044    tracking=3.044    overshoot= 0.0%

   XML snippet:
     <KP>1360</KP>
     <KI>271.9</KI>
     <KD>0</KD>
     <max_torque>100</max_torque>

The ``XML`` line evaluates the gains currently in the vehicle definition, if its controller
is a twist PID. The XML snippet can be pasted into the vehicle ``<controller>`` block.

Supported Vehicle Types
-----------------------

* **Differential drive** (``differential``, ``differential_3_wheels``, ``differential_4_wheels``)
  -- tunes the ``twist_pid`` controller. For skid-steer robots (4 wheels), it also tunes
  ``wheel_separation_multiplier``.
* **Ackermann steering** (``ackermann``) -- tunes the ``twist_front_steer_pid`` controller.
* **Ackermann with drivetrain** (``ackermann_drivetrain``) -- tunes its
  ``twist_front_steer_pid`` controller.

If the vehicle definition uses another controller (e.g. ``twist_ideal``), the twist PID one is
installed for the tests. Other parameters in the XML (e.g. steering rate limits, drivetrain
settings) are kept.
