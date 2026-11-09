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

``--trajectory`` accepts two formats, selected automatically from the file
extension (``--trajectory-format`` overrides the guess):

- **tum**: a `TUM-format
  <https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats>`_ text
  file, one ``timestamp tx ty tz qx qy qz qw`` line per pose, full SE(3),
  epoch-seconds timestamps. Poses in between are interpolated (linear
  position + SLERP orientation by default).

- **waypoints2d**: a 2D ``(t, x, y)`` waypoint list, either a plain-text
  file (one ``t x y`` line per waypoint, ``#`` comments and blank lines
  ignored) or an XML file with one ``<waypoint t="" x="" y=""/>`` tag per
  waypoint (attribute order does not matter). Heading is derived from the
  path tangent, and ``z``/pitch/roll are derived from the world's ground
  surface at 4 probe points under the vehicle footprint
  (``--footprint-lx``/``--footprint-ly``), i.e. terrain-following.

Sensors
-------

- **3D LiDAR**: one sweep per ``lidar3d`` sensor aboard the vehicle,
  reusing that sensor's own XML-configured parameters (ring count/FOV or
  explicit ``vertical_ray_angles``, azimuth resolution, sensor period,
  min/max range, noise, intensity). Two shutter modes:

  - ``--shutter rolling`` (default): a realistic rotating LiDAR. Each
    azimuth column fires at its own time, ``col * period / horz_nrays``
    after the sweep start, and is ray-cast from the vehicle pose
    interpolated at exactly that instant. Each point's ``t`` field holds
    that relative time, in ``[0, period)`` (e.g. ``[0, 0.1)`` for a 10 Hz
    sensor), and the point is expressed in the sensor frame at that same
    instant, just like a real LiDAR driver outputs it: the cloud is
    motion-skewed, and consumers can de-skew it using ``t``.
  - ``--shutter global``: one pose (the sweep-start one) for the whole
    sweep; every point's ``t`` field is 0.

  Points are emitted as a ``CObservationPointCloud`` /
  ``CGenericPointsMap`` with ``t``/``ring``/(optional)``intensity`` fields.
  The observation timestamp is the sweep start time, and ``t`` is relative
  to it.

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

Realistic LiDAR errors
----------------------

On top of the Gaussian ``range_std_noise``, a ``lidar3d`` sensor can enable a
realistic error model, applied by this tool only (the interactive simulator
ignores it). All parameters default to 0 (disabled), which leaves the output
unchanged; ``--noiseless`` disables them too, and ``--noise-scale`` scales the
noise and calibration sigmas:

- ``range_std_noise_per_meter``: range noise std growing linearly with range.
- ``beam_divergence_deg``: full beam divergence. On a surface hit at
  incidence angle :math:`\theta`, the footprint spreads the return over a
  range interval of width :math:`r \, \delta \tan\theta`, adding its uniform
  spread to the noise.
- ``max_incidence_deg``: no return beyond this incidence angle.
- ``mixed_pixel_probability``: where neighbor rays hit different surfaces
  (one lies off the other's tangent plane), the return is, with this
  probability, a range between both, as in real sensors at depth edges.
- ``beam_elevation_std_deg``, ``beam_azimuth_std_deg``,
  ``beam_range_bias_std``: per-beam calibration errors, drawn once per run
  from ``calibration_seed``. Rays sample the scene along the perturbed
  direction, while points are reported along the nominal one.

The stock 3D LiDAR definitions expose them as the XML variables
``sensor_range_std_noise_per_meter``, ``sensor_beam_divergence_deg``,
``sensor_max_incidence_deg``, ``sensor_mixed_pixel_probability``,
``sensor_beam_elevation_std_deg``, ``sensor_beam_azimuth_std_deg``,
``sensor_beam_range_bias_std`` and ``sensor_calibration_seed`` (as
``<include>`` attributes or environment variables).

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
       --shutter rolling \
       -o /tmp/demo_dataset.rawlog

See ``mvsim_tutorial/demo_dataset_gen.world.xml`` and the accompanying
``.trajectory.tum`` file for a minimal working example (a ground plane,
some cylindrical pillars and boxes, a Velodyne VLP-16 and an IMU).

Full option list: ``mvsim-dataset-gen --help``.
