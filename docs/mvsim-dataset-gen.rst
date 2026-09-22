mvsim-dataset-gen
===================

``mvsim-dataset-gen`` is an offline CLI tool that generates simulated sensor
datasets (MRPT ``.rawlog`` files) from a world XML and a **prescribed**
ground-truth trajectory, with no interactive GUI and no OpenGL rendering
anywhere in the pipeline. Its 3D LiDAR is simulated by exact mathematical
ray casting, not by rasterizing depth images (unlike the interactive
``lidar3d`` sensor used by ``mvsim launch``), so it has no depth-buffer
quantization and no GPU/EGL dependency.

.. note::
   The vehicle's pose is **prescribed**, not simulated: it is read directly
   from the input trajectory at each sensor firing time. There is no
   controller, no Box2D physics stepping, and no tracking error — the
   trajectory *is* the ground truth.

Supported world geometry
-------------------------

Only analytic geometry can be ray-traced:

- ``<block>`` shapes: an explicit ``<shape>`` polygon, or
  ``<geometry type="cylinder|sphere|box">`` (exact primitives), or
  ``<geometry type="ramp"|"semi_cylinder_bump">`` (tessellated to
  triangles). A block with no ``<geometry>`` tag falls back to its 2D
  collision footprint (``<shape>``, the default unit square, or
  ``<shape_from_visual/>``'s bounding box) extruded between ``zmin``/``zmax``
  — this is an approximation when the footprint came from
  ``<shape_from_visual/>``: the block's exact visual mesh is never
  ray-traced, only its footprint's bounding prism.
- ``<element class="horizontal_plane">`` and
  ``<element class="vertical_plane">`` (door/window openings are modeled as
  full through-gaps).
- ``<element class="elevation_map">``.
- ``<element class="ground_grid">``, ``<element class="sky_box">`` and
  ``<element class="property_region">`` are purely visual/non-geometric and
  are silently skipped.

Anything else (occupancy grids, point clouds, additional vehicles besides
the one carrying the sensors) aborts the run with a clear error, unless
``--allow-unsupported`` is given, in which case it is skipped with a
warning instead.

Trajectory input
-----------------

A `TUM-format <https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats>`_
text file (``--trajectory``): one ``timestamp tx ty tz qx qy qz qw`` line
per pose, full SE(3), epoch-seconds timestamps. Poses in between are
interpolated (linear position + SLERP orientation by default).

.. note::
   2D ``(x, y, phi)`` trajectory input with terrain-following (deriving
   ``z``/roll/pitch from the world's ground surface) is planned but not yet
   implemented; only ``.tum`` files are accepted for now.

Sensors
-------

- **3D LiDAR**: one sweep per ``lidar3d`` sensor aboard the vehicle,
  reusing that sensor's own XML-configured parameters (ring count/FOV or
  explicit ``vertical_ray_angles``, azimuth resolution, sensor period,
  min/max range, noise, intensity). Two shutter modes:

  - ``--shutter global`` (default): one pose for the whole sweep; every
    point's ``t`` field is 0.
  - ``--shutter rolling``: the vehicle pose is re-interpolated once per
    azimuth column (the usual "each column fires at a time" model of real
    rotating LiDARs), producing genuinely motion-skewed points; ``t`` is
    the column's firing offset from the sweep start.

  In both modes, points are emitted as a ``CObservationPointCloud`` /
  ``CGenericPointsMap`` with ``t``/``ring``/(optional)``intensity`` fields,
  expressed in the sensor frame *at the sweep-start pose* — the usual
  convention for de-skewing consumers.

- **IMU**: one sample per ``imu`` sensor, computed by differentiating the
  prescribed trajectory (an exact SO(3) log for angular velocity, a
  finite-difference second derivative of position for proper
  acceleration), reusing the same noise model (white noise + bias random
  walk) as the interactive simulator.

- **Wheel odometry**: dead-reckoned from the trajectory's planar
  projection, with noise proportional to the motion magnitude
  (``--odom-rate``, ``--odom-trans-noise-rel``, ``--odom-rot-noise-rel``).
  mvsim's world XML has no per-vehicle ``<odometry>`` sensor tag (the
  interactive simulator derives it from the stepped Box2D wheel state
  instead), so this tool synthesizes it directly from the trajectory.

All 3 sensor streams are merged into the ``.rawlog`` in strict global
chronological order.

Noise and reproducibility
--------------------------

Per-sensor noise sigmas come from the world XML by default (the same
values the interactive simulator would use). ``--noiseless`` forces every
sigma to zero, for generating an ideal ground-truth dataset;
``--noise-scale k`` scales every sigma instead. ``--seed`` fixes the RNG
seed; the same seed always produces a byte-identical ``.rawlog``.

Output
------

- ``<output>.rawlog``: all observations.
- ``<output>.gt.tum``: ground-truth vehicle pose, sampled at the LiDAR
  rate.
- ``<output>.<sensor_label>.gt.tum``: ground-truth pose of each LiDAR
  sensor (vehicle pose composed with the sensor's mount offset) — this is
  what trajectory-error tooling (e.g. ``evo``) actually wants to compare
  against.

Example
-------

.. code-block:: console

   $ mvsim-dataset-gen mvsim_tutorial/demo_dataset_gen.world.xml \
       --trajectory mvsim_tutorial/demo_dataset_gen.trajectory.tum \
       -o /tmp/demo_dataset.rawlog

See ``mvsim_tutorial/demo_dataset_gen.world.xml`` and the accompanying
``.trajectory.tum`` file for a minimal working example (a ground plane,
some cylindrical pillars and boxes, a Velodyne VLP-16 and an IMU).

Full option list: ``mvsim-dataset-gen --help``.
