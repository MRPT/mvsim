.. _world_joints:

Joints between objects
--------------------------------------------

MVSim supports **joint constraints** between any two ``Simulable`` objects
(vehicles, blocks, or a mix of both). Joints are backed by Box2D joint
primitives and are solved together with the rest of the physics each time step.

Typical use-cases include tractor-trailer articulation, tow ropes, coupled
robots, and passive trailers pulled by a powered vehicle.

.. note::

   ``<joint>`` tags must appear **after** all referenced ``<vehicle>`` and
   ``<block>`` tags in the world file, because the bodies need to exist before
   they can be connected.


Joint types
^^^^^^^^^^^^^

Two joint types are currently available, selected via the ``type`` attribute:

``distance`` — rope / distance constraint
  A distance joint (``b2DistanceJoint``) that allows the two anchor points to
  be *closer* than ``max_length`` but never *farther* apart, like a rope or
  cable. Setting ``stiffness`` and ``damping`` to zero (the default) gives a
  rigid length limit; non-zero values make the constraint springy.

``revolute`` — pin / hinge constraint
  A revolute joint (``b2RevoluteJoint``) that forces two anchor points to
  remain coincident and allows only relative rotation around that shared point.
  Optional angle limits prevent excessive articulation (e.g. jack-knifing).


Common attributes
^^^^^^^^^^^^^^^^^^

Every ``<joint>`` tag requires the following attributes:

- **type**: ``"distance"`` or ``"revolute"``.
- **body_a**: Name of the first ``Simulable`` object (its ``name`` attribute).
- **body_b**: Name of the second ``Simulable`` object.
- **anchor_a**: ``"x y"`` offset in **body_a**'s local coordinate frame.
- **anchor_b**: ``"x y"`` offset in **body_b**'s local coordinate frame.


Distance joint attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^

These attributes are only used when ``type="distance"``:

- **max_length** *(required)*: Maximum distance (metres) between anchors.
- **min_length** *(optional, default 0)*: Minimum distance. Zero means the
  bodies can touch.
- **stiffness** *(optional, default 0)*: Spring stiffness (N/m). Zero gives a
  rigid limit.
- **damping** *(optional, default 0)*: Damping coefficient (N·s/m).


Revolute joint attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^

These attributes are only used when ``type="revolute"``:

- **enable_limit** *(optional, default false)*: Whether angular limits are
  enforced.
- **lower_angle_deg** *(optional, default 0)*: Lower angular limit in degrees
  (only if ``enable_limit="true"``).
- **upper_angle_deg** *(optional, default 0)*: Upper angular limit in degrees.


Visualization
^^^^^^^^^^^^^^^

Joints are rendered in the 3-D GUI automatically:

- **Distance joints**: Yellow line between the two world-space anchor points.
- **Revolute joints**: Red line between anchors plus a small cross marker at
  the pivot.


Examples
^^^^^^^^^^

**Rope between two blocks:**

.. code-block:: xml

   <joint type="distance"
       body_a="cargo_a" anchor_a="-0.5 0.0"
       body_b="cargo_b" anchor_b=" 0.5 0.0"
       max_length="3.0"
   />

**Pin joint for a tractor-trailer (with angle limits):**

.. code-block:: xml

   <joint type="revolute"
       body_a="tractor"  anchor_a="-1.5 0.0"
       body_b="trailer"  anchor_b=" 1.0 0.0"
       enable_limit="true"
       lower_angle_deg="-75"
       upper_angle_deg="75"
   />

**Springy tow line:**

.. code-block:: xml

   <joint type="distance"
       body_a="truck"   anchor_a="-1.0 0.0"
       body_b="caravan" anchor_b=" 1.2 0.0"
       max_length="2.5"
       stiffness="500.0"
       damping="50.0"
   />


Passive (unpowered) trailers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A common pattern is a trailer that has no motor of its own and is pulled
entirely through a joint. To achieve this, define the trailer as a normal
vehicle with a ``raw`` controller and leave all torque values at zero:

.. code-block:: xml

   <vehicle:class name="trailer_class">
     <dynamics class="differential_4_wheels">
       <chassis mass="300" zmin="0.05" zmax="0.6">
         <!-- shape polygon ... -->
       </chassis>
       <!-- define 4 passive wheels here -->
       <controller class="raw">
         <l_torque>0</l_torque>
         <r_torque>0</r_torque>
       </controller>
     </dynamics>
   </vehicle:class>

   <vehicle name="trailer" class="trailer_class">
     <init_pose>-2.8 0 0</init_pose>
   </vehicle>

   <!-- Connect tractor rear to trailer front -->
   <joint type="revolute"
       body_a="tractor" anchor_a="-0.8 0.0"
       body_b="trailer" anchor_b=" 2.0 0.0"
       enable_limit="true"
       lower_angle_deg="-75" upper_angle_deg="75"
   />

.. tip::

   Position both bodies so their anchor points overlap at :math:`t=0`; this
   avoids a sudden impulse on the first simulation step.