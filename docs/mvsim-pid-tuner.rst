.. _mvsim-pid-tuner:

mvsim-pid-tuner
===============

``mvsim-pid-tuner`` is a command-line tool for automatic PID parameter tuning of MVSim
vehicle controllers. It identifies the vehicle's dynamic response and proposes optimal
PID gains using the IMC (Internal Model Control) tuning method.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

Overview
--------

Tuning PID controllers for simulated vehicles can be tedious. This tool automates the
process by:

1. Running an **open-loop step response** test (constant torque applied to wheels)
2. **Identifying the plant model** as a first-order system: :math:`G(s) = K / (\tau s + 1)`
3. Computing **optimal PID parameters** using IMC tuning rules
4. **Validating** the proposed parameters with a closed-loop step-up/step-down simulation

Algorithm
---------

**Phase 1: Plant Identification**

A constant torque is applied to all wheels (via a ``raw`` controller) and the average
wheel velocity is recorded over time. The steady-state velocity :math:`v_{ss}` is
computed from the last 20% of samples, and the plant gain and time constant are:

.. math::

   K = \frac{v_{ss}}{\tau_{applied}}, \quad \tau = \text{time to reach } 63.2\% \text{ of } v_{ss}

**Phase 2: IMC PID Tuning**

For a first-order plant, the IMC method yields a PI controller:

.. math::

   K_P = \frac{\tau}{K \cdot \tau_{cl}}, \quad K_I = \frac{K_P}{\tau}, \quad K_D = 0

where :math:`\tau_{cl} = \tau \cdot \alpha` is the desired closed-loop time constant and
:math:`\alpha` is the aggressiveness factor (smaller = faster response).

The ``max_torque`` is set to 80% of the estimated friction limit to avoid wheel slip.

**Phase 3: Closed-Loop Validation**

The proposed PID parameters are tested with a step-up to 1.0 m/s (3 seconds) followed
by a step-down to 0 m/s (3 seconds). The tool reports:

* Rise time (time to 90% of setpoint)
* Settling time (time to enter 2% band)
* Overshoot percentage
* Steady-state error
* Stop settling time and rebound

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
   :widths: 20 15 65

   * - Flag
     - Default
     - Description
   * - ``-t``, ``--torque``
     - auto
     - Test torque (Nm) for open-loop identification. Default: 50% of estimated friction limit.
   * - ``-d``, ``--duration``
     - 5.0
     - Open-loop step duration in seconds.
   * - ``-s``, ``--sim-step``
     - 0.001
     - Simulation time step in seconds.
   * - ``-a``, ``--aggressiveness``
     - 0.25
     - Closed-loop aggressiveness factor. Range: 0.1 (very aggressive) to 1.0 (conservative).

Example
-------

.. code-block:: bash

   mvsim-pid-tuner definitions/small_robot.vehicle.xml small_robot

Sample output:

.. code-block:: text

   ======================================
    MVSim PID Auto-Tuner
   ======================================

   --- Phase 1: Open-loop plant identification ---
   Vehicle: small_robot (2 wheels)
     Wheel 0: pos=(0.000, 0.500) R=0.200 mass=4.00
     Wheel 1: pos=(0.000, -0.500) R=0.200 mass=4.00
     Chassis mass: 15.0 kg
     Estimated max friction torque per wheel: 18.10 Nm (mu=0.8)
     Test torque: 9.05 Nm

   === Open-Loop Step Response Analysis ===
   Applied torque:        9.050 Nm
   Steady-state velocity: 1.2345 m/s
   Plant gain K:          0.1364 (m/s)/Nm
   Time constant τ:       0.1500 s

   --- Phase 2: IMC PID tuning (aggressiveness=0.25) ---

   Proposed PID parameters:
     KP:         29.3200
     KI:         195.4667
     KD:         0.0000
     max_torque: 14.48 Nm

   --- Phase 3: Closed-loop validation ---
   Step-up (0 -> 1.0 m/s):
     Rise time (90%):       0.120 s
     Settling time (2%):    0.250 s
     Overshoot:             2.5 %
     Steady-state error:    0.0012 m/s

   Step-down (1.0 -> 0 m/s):
     Settling time (2%):    0.300 s
     Rebound:               none

   --- Assessment ---
   All metrics look good!

The tool prints an XML snippet with the proposed PID values that can be pasted directly
into the vehicle's ``<controller>`` block.

Supported Vehicle Types
-----------------------

* **Differential drive** (``differential``, ``differential_3_wheels``, ``differential_4_wheels``)
  -- tunes the ``twist_pid`` controller
* **Ackermann steering** (``ackermann``) -- tunes the ``twist_front_steer_pid`` controller
