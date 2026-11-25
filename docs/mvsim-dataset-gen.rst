mvsim-dataset-gen
===================

``mvsim-dataset-gen`` is an offline CLI tool that generates simulated sensor
datasets (MRPT ``.rawlog`` files: 3D LiDAR, IMU, GNSS, wheel odometry and
cameras, with the exact ground truth of the trajectory and of every sensor)
from a world XML and a **prescribed**
ground-truth trajectory, with no interactive GUI. Its 3D LiDAR is simulated
by exact mathematical ray casting, not by rasterizing depth images (unlike
the interactive ``lidar3d`` sensor used by ``mvsim launch``), so it has no
depth-buffer quantization and no GPU/EGL dependency. Cameras, if the
vehicle has any, are rendered with OpenGL off-screen (EGL: a GPU, or Mesa's
software renderer without one).

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

Moving blocks
-------------

The scene is static by default. To have objects that move during the run
(for example, traffic around the sensor vehicle), give each of them its own
trajectory with the repeatable option:

.. code-block:: console

   --moving-block <block name>=<trajectory.tum>

- ``<block name>`` is the ``name`` of a ``<block>`` of the world XML, with
  any of the block geometries above. It is left out of the static scene.
- The trajectory is a TUM file (``timestamp tx ty tz qx qy qz qw``, in the
  same epoch-seconds clock as ``--trajectory``) with the pose of the block's
  own frame in the world along time, full SE(3). It replaces the block's
  ``<init_pose>``. Before its first pose and after its last one, the block
  stays at that pose.
- Several blocks may share one trajectory file, e.g. a vehicle made of two
  boxes.

Each LiDAR ray returns its nearest hit over the static scene and all moving
blocks. With ``--shutter rolling``, each azimuth column sees every moving
block at the block's pose at that column's firing time; with
``--shutter global``, at the sweep start time. The LiDAR noise and error
models and the semantic labels apply to moving blocks like to the static
scene. IMU and odometry do not depend on them.

An unknown block name, a block given twice, an ``<intangible>`` block (those
are not ray-traced), or a trajectory file that cannot be read or has fewer
than two poses abort the run with an error.

.. code-block:: console

   $ mvsim-dataset-gen world.xml --trajectory ego.tum -o out.rawlog \
       --moving-block car1_body=car1.tum --moving-block car1_cabin=car1.tum \
       --moving-block car2=car2.tum

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
  ``CGenericPointsMap`` with ``t``/``ring``/(optional)``intensity`` fields,
  and ``label``/``instance`` with ``--labels`` (see below).
  The observation timestamp is the sweep start time, and ``t`` is relative
  to it.

- **IMU**: one sample per ``imu`` sensor, computed by differentiating the
  prescribed trajectory (an exact SO(3) log for angular velocity, a
  finite-difference second derivative of position for proper
  acceleration), reusing the same error model as the interactive simulator:
  white noise, bias random walk, and the deterministic errors (constant
  bias, scale factors and axis misalignment, see the IMU section of
  :doc:`sensors`). Each IMU has its own noise stream. With a bias random
  walk, the bias of every sample is written to
  ``<output>.<sensor_label>.bias.txt``.

- **GNSS**: one reading per ``gnss`` sensor and period, at the antenna
  (the sensor pose on the vehicle), with the sensor's fix type, the
  reported ENU covariance from its noise parameters, and its ``<event>``
  episodes (quality changes, outages, position jumps; see the GNSS section
  of :doc:`sensors`), whose ``start``/``end`` times count from the first
  pose of the trajectory. A jump with the ``rtk_fixed`` fix type is a false
  fix. Readings are ``CObservationGPS`` with an NMEA GGA message (its
  ``fix_quality`` is 4 for RTK fixed, 5 for RTK float) and ``fix_type``.
  The world needs a ``<georeference>``.

- **Wheel odometry**: dead-reckoned from the trajectory's planar
  projection, with noise proportional to the motion magnitude
  (``--odom-rate``, ``--odom-trans-noise-rel``, ``--odom-rot-noise-rel``).
  mvsim's world XML has no per-vehicle ``<odometry>`` sensor tag (the
  interactive simulator derives it from the stepped Box2D wheel state
  instead), so this tool synthesizes it directly from the trajectory.

- **Cameras** (``camera`` and ``rgbd_camera`` sensors): see below.

All sensor streams are merged into the ``.rawlog`` in strict global order
of their timestamps.

Cameras
-------

``camera`` (RGB) and ``rgbd_camera`` (RGB and/or depth) sensors are
rendered off-screen with OpenGL through EGL, at the exact times of the
trajectory, from a scene of the world elements and blocks (never the
vehicle carrying the sensors; moving blocks at their poses), lit by the
world ``<lights>``. ``--no-cameras`` skips them. Images use the sensor
options of the interactive simulator: intrinsics, clip distances, the
``plumb_bob`` lens distortion and Gaussian pixel noise (both need MRPT >=
3.6.0), and for depth images the resolution, the noise
(``depth_noise_sigma``) and the systematic depth errors (``depth_error_*``,
see :doc:`sensors`). Noise uses a stream of its own per camera, so cameras
never change the other sensors' data, and ``--noiseless`` removes the noise
and the depth errors.

- The intrinsics follow the OpenCV / ROS ``camera_info`` convention: the
  center of pixel (u, v) is at image coordinates (u, v).
- Depths are along the optical axis, computed in double precision from the
  depth buffer.
- **Rolling shutter** (optional, RGB): ``rolling_shutter_line_time`` is the
  time between the exposures of consecutive rows [s] (``rgb_`` prefix in
  ``rgbd_camera``). Row ``i`` of a frame stamped ``t`` is exposed at
  ``t + i * line_time``. The image is rendered as
  ``rolling_shutter_bands`` (default 8) bands of rows, each from the pose at
  the time of its middle row: an approximation whose error is half a band's
  readout time.

Images are written next to the rawlog, in ``<output name>_Images/`` (the
images directory MRPT looks for), and referenced from the observations by
file name: RGB images as PNG (``<label>_<index>.png``), depth images in
MRPT's external range image format (``<label>_<index>_depth.bin``: a
zstd-compressed matrix of ``uint16`` depths in units of
``depth_resolution``, 0 for no return; ``CObservation3DRangeScan::load()``
reads it). RGB cameras give ``CObservationImage``, depth cameras
``CObservation3DRangeScan``. PNG files are encoded by ``--image-threads``
worker threads.

Clock offsets
-------------

Real sensors stamp their data with some delay or advance. To simulate it,
``--clock-offset <sensor label>=<seconds>`` (repeatable) shifts the
timestamps of one stream: each observation is sampled at its true time
``t`` and stamped ``t + offset`` (positive: stamped late). The data itself
does not change, and the LiDAR per-point ``t`` field stays relative to the
observation timestamp: streams are generated in sampling time order and
written in stamp order, so the offsets change the stamps only. The label of
the wheel odometry stream is ``odom``, which no sensor may have. Ground-truth
files keep the true times.

.. code-block:: console

   $ mvsim-dataset-gen world.xml --trajectory ego.tum -o out.rawlog \
       --clock-offset imu1=0.004 --clock-offset gnss1=0.1

Noise and reproducibility
--------------------------

Per-sensor noise sigmas come from the world XML by default (the same
values the interactive simulator would use). ``--noiseless`` forces every
sigma to zero, for generating an ideal ground-truth dataset: it also removes
the LiDAR per-beam errors, the IMU constant bias, scale factors and
misalignment, and the GNSS event offsets (GNSS fix types, reported
covariances and outages stay). ``--noise-scale k`` scales every sigma
instead, and the GNSS event offsets, but not the IMU deterministic errors.
``--seed`` fixes the RNG seed; the same seed always produces a
byte-identical ``.rawlog``.

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
- ``echo_separation``: enables a beam model instead (it needs
  ``beam_divergence_deg`` > 0, and ``mixed_pixel_probability`` must be 0).
  Each ray samples its footprint with seven sub-rays: its axis and six rays
  on the cone of half the divergence, weighted as a Gaussian beam whose full
  width at half maximum is the divergence (the axis carries a quarter of the
  energy). Their hits, in range order, form echoes: a hit joins the previous
  one if both lie on one surface (each within 5 cm of the other's tangent
  plane), or if they are closer than this distance [m] along the beam, as the
  pulses they return overlap. An echo's range is the mean of its hits
  weighted by the received power :math:`\cos\theta / r^2`, and the
  strongest echo is the return. Its range spread replaces the
  :math:`r \, \delta \tan\theta` term (up to its value at 80 deg of
  incidence, and at most 0.15 m: real sensors measured at any incidence are
  not noisier), drawn within the echo's range interval, so that an object
  narrower than the footprint spreads its returns only over its own depth.
  At a depth edge, two surfaces closer than the separation give a return
  between both, whose mean range stays the same in every sweep for a still
  sensor; farther apart, the return comes from the stronger one. The point
  is still reported along the ray's axis, and the axis must hit something
  for a return; ``max_incidence_deg`` drops the sub-ray hits beyond it.
- ``detection_range``: the range [m] at which a surface seen at normal
  incidence is still detected: a return needs
  :math:`\cos\theta / r^2 \geq 1 / R^2` (with the beam model, the echo's
  power, the sum over its hits of their share of the beam times
  :math:`\cos\theta / r^2`). 0 disables it.
- ``beam_elevation_std_deg``, ``beam_azimuth_std_deg``,
  ``beam_range_bias_std``: per-beam calibration errors, drawn once per run
  from ``calibration_seed``. Rays sample the scene along the perturbed
  direction, while points are reported along the nominal one.

The stock 3D LiDAR definitions expose them as the XML variables
``sensor_range_std_noise_per_meter``, ``sensor_beam_divergence_deg``,
``sensor_max_incidence_deg``, ``sensor_mixed_pixel_probability``,
``sensor_echo_separation``, ``sensor_detection_range``,
``sensor_beam_elevation_std_deg``, ``sensor_beam_azimuth_std_deg``,
``sensor_beam_range_bias_std`` and ``sensor_calibration_seed`` (as
``<include>`` attributes or environment variables).

Semantic labels
---------------

Any block or world element (``horizontal_plane``, ``vertical_plane``,
``elevation_map``) may carry two optional integer tags in ``[0, 65535]``:

- ``<semantic_class>``: its class (0, the default, means unlabeled). The
  meaning of each value is up to the world author.
- ``<instance_id>``: its instance (0, the default, means none). Several
  objects may share one, e.g. the trunk and foliage blocks of one tree.

Tags in a ``<block:class>`` apply to every block of that class, and each
block may override them:

.. code-block:: xml

   <block:class name="pillar">
     <geometry type="cylinder" radius="0.4" length="3.0" />
     <semantic_class>2</semantic_class>
   </block:class>
   <block class="pillar"> <init_pose>5 3 0</init_pose> <instance_id>1</instance_id> </block>

With ``--labels``, every LiDAR point gets two ``uint16`` fields, ``label``
and ``instance``: the tags of the object its ray hit. Labels never change
the samples: the points are the same as without ``--labels``, for the same
seed. A mixed return (see ``mixed_pixel_probability``) keeps the labels of
the surface its own ray hit, unless ``--mixed-return-label N`` gives all of
them class ``N`` and instance 0 instead. With the beam model, a return has
the labels of the strongest hit of its echo, and an echo that joins hits on
different surfaces is a mixed return.

Output
------

- ``<output>.rawlog``: all observations.
- ``<output>.gt.tum``: ground-truth vehicle pose, sampled at the LiDAR
  rate.
- ``<output>.<sensor_label>.gt.tum``: ground-truth pose of each LiDAR
  sensor (vehicle pose composed with the sensor's mount offset) — this is
  what trajectory-error tooling (e.g. ``evo``) actually wants to compare
  against.
- ``<output>.gt.yaml``: ground truth of every sensor (see below).
- ``<output name>_Images/``: camera images (see Cameras).
- ``<output>.<sensor_label>.bias.txt``: for an IMU with a bias random walk,
  the bias of each sample: ``t`` and the gyroscope and accelerometer biases
  (x, y, z).

Sensor ground truth
-------------------

``<output>.gt.yaml`` records, for every generated stream, the exact values
in effect: the sensor pose on the vehicle, the clock offset, the rate, and
the sensor intrinsics and errors, including those drawn at random (LiDAR
per-beam errors). Times are seconds in the clock of the trajectory. The
format, ``mvsim/dataset_gt/1``:

.. code-block:: yaml

   format: mvsim/dataset_gt/1
   world: world.xml            # as given on the command line
   vehicle: r1
   base_frame: base_link       # the vehicle frame
   rawlog: out.rawlog
   vehicle_ground_truth: out.gt.tum
   time: {start: 100, end: 280}
   lidar_shutter: rolling
   seed: 0
   noiseless: false
   noise_scale: 1
   georeference: {...}         # if the world has one
   sensors:
     <sensor label>:
       type: lidar3d           # lidar3d | imu | gnss | odometry | camera
                               # | depth_camera | rgbd_camera
       frame_id: <sensor label>
       pose_in_vehicle:        # T_vehicle_sensor
         xyz: [x, y, z]
         quat_xyzw: [qx, qy, qz, qw]
         ypr_deg: [yaw, pitch, roll]
       clock_offset_s: 0       # stamp minus true sampling time
       rate_hz: 10
       intrinsics: {...}       # one block per type, below

``intrinsics`` of each sensor type:

- ``lidar3d``: ``rings``, ``horz_nrays``, ``min_range``, ``max_range``,
  ``elevations_deg`` (nominal, ring 0 first, ascending), ``noise`` (the
  error model parameters in effect), ``calibration_seed``,
  ``beam_error_std`` and ``beam_errors``, one row per ring:
  ``{ring, d_elevation_deg, d_azimuth_deg, d_range_m}``. The angles are the
  perturbed (sampled) ray minus the nominal one, along which points are
  reported; ``d_range_m`` is the reported range minus the true one. Without
  per-beam errors, they are all zero.
- ``imu``: ``gyroscope`` and ``accelerometer``, each with ``bias`` (the
  constant bias), ``scale`` (diagonal), ``misalignment`` (the full 3x3
  matrix M, zero diagonal), ``sensitivity_matrix`` (diag(scale) (I + M),
  which multiplies the true value) and ``noise``
  (``white_noise_std``, ``random_walk_std``); and ``bias_file`` if written.
- ``gnss``: the antenna is ``pose_in_vehicle.xyz``. ``quality`` (fix type
  outside the episodes: ``no_fix``, ``single``, ``dgps``, ``rtk_float``,
  ``rtk_fixed``), ``horizontal_std``, ``vertical_std``, ``noise_scale``, and
  ``episodes``: ``{t_start, t_end, quality, horizontal_std, vertical_std,
  injected_error_m}``, the intervals ``[t_start, t_end)`` (absolute times)
  where any of them differs from the nominal values: the values in effect,
  from all the ``<event>`` tags covering the interval (overlapping events
  give one episode per combination), ``quality: outage`` for outages, and
  the ENU position error added to the readings.
- ``odometry`` (the wheel odometry stream, ``odom``, at the vehicle frame):
  ``trans_noise_rel``, ``rot_noise_rel``.

- ``camera``: the pose is that of the optical frame (+Z forward, +X right,
  +Y down). ``width``, ``height``, ``fx``, ``fy``, ``cx``, ``cy`` (OpenCV
  convention), ``distortion: {model: none|plumb_bob, coeffs: [k1, k2, p1,
  p2, k3]}``, ``clip_m``, ``rolling_shutter_line_time_s`` (0: global
  shutter; with ``rolling_shutter_bands`` otherwise), ``image_noise_std``
  and ``files`` (the path pattern of the images, relative to the dataset).
- ``depth_camera`` (an ``rgbd_camera`` without RGB): the same keys, but
  ``depth_unit_m``, ``depth_noise_std_m`` and ``depth_errors: {scale,
  offset_m, radial: [k1, k2]}`` instead of the rolling shutter and pixel
  noise.
- ``rgbd_camera``: ``rgb`` and ``depth`` sub-blocks, each with its own
  ``pose_in_vehicle`` (optical frames) and the keys above; the sensor
  ``pose_in_vehicle`` is that of the depth optical frame.

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
