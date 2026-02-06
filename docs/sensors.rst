Sensors
===================

This page describes the predefined sensors and their parameters.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Sensor definition and common parameters
-----------------------------------------

A sensor is attached to a vehicle by adding an XML block with the 
tag name ``<sensor>.... </sensor>`` inside:

- the ``<vehicle>...</vehicle>`` that instantiates a particular robot, or
- the ``<vehicle:class>`` to have *all* vehicles of that type with the same sensors installed.

All sensors, via the common base C++ class ``mvsim::SensorBase``, have the 
common parameters below, with their meaning explained in the XML comments.

.. code-block:: xml

	<!-- 
	 ``class``: One of the registered sensor classes.
	 ``name``: A name for the sensor, must be unique for each robot. If not provided,
	           an automatic name will be generated. The name is important since it is
	           used when publishing the sensor ROS or ZMQ topics.
	-->
	<sensor class="camera" name="camera1">
		<!-- Period (in seconds) between each sensor observation.
		     Mathematical expressions can be used with $f{} to specify rates in Hz.
		 -->
		<sensor_period>0.1</sensor_period>
		<!-- <sensor_period>$f{1/20.0}</sensor_period> -->

		<!-- See notes below -->
		<visual>
			...
		</visual>

		<!-- Publish sensor on MVSIM ZMQ topic? (Note, **not** related to ROS at all) -->
		<!-- <publish>
			<publish_topic>/${PARENT_NAME}/${NAME}</publish_topic>
		</publish> -->
	</sensor>

The ``<visual>`` tag is explained in the :ref:`world_visual_object`, and
the ``<publish>`` tag in :ref:`world_simulable`, since those tags are common
to sensors and many other entities.

Also, note that you can use the XML-level variables ``${PARENT_NAME}`` and 
``${NAME}`` anywhere inside the sensor definition tag to refer to the parent vehicle and own sensor names,
respectively.

Each sensor ``class`` has its own additional parameters, listed in the next sections.


3D LiDARs
-----------------------------------------

HELIOS 32 (26 deg FOV)
##########################

.. image:: https://mrpt.github.io/imgs/mvsim-lidar-helios32-26.png
   :width: 100%
   :alt: sensor preview in MVSim

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/helios-32-FOV-26.sensor.xml"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rate="10.0"
		/>


.. dropdown:: All parameters available in helios-32-FOV-26.sensor.xml

   File: `mvsim_tutorial/definitions/helios-32-FOV-26.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/helios-32-FOV-26.sensor.xml>`_

   .. literalinclude:: ../definitions/helios-32-FOV-26.sensor.xml
      :language: xml

|

HELIOS 32 (31 deg FOV)
##########################

.. image:: https://mrpt.github.io/imgs/mvsim-lidar-helios32-31.png
   :width: 100%
   :alt: sensor preview in MVSim

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/helios-32-FOV-31.sensor.xml"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rate="10.0"
		/>


.. dropdown:: All parameters available in helios-32-FOV-31.sensor.xml

   File: `mvsim_tutorial/definitions/helios-32-FOV-31.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/helios-32-FOV-31.sensor.xml>`_

   .. literalinclude:: ../definitions/helios-32-FOV-31.sensor.xml
      :language: xml

|

HELIOS 32 (70 deg FOV)
##########################

.. image:: https://mrpt.github.io/imgs/mvsim-lidar-helios32-70.png
   :width: 100%
   :alt: sensor preview in MVSim

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/helios-32-FOV-70.sensor.xml"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rate="10.0"
		/>

.. dropdown:: All parameters available in helios-32-FOV-70.sensor.xml

   File: `mvsim_tutorial/definitions/helios-32-FOV-70.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/helios-32-FOV-70.sensor.xml>`_

   .. literalinclude:: ../definitions/helios-32-FOV-70.sensor.xml
      :language: xml

|

OUSTER OS1
##########################

.. image:: https://mrpt.github.io/imgs/mvsim-lidar-ouster-os1.png
   :width: 100%
   :alt: sensor preview in MVSim

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/ouster-os1.sensor"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_period_sec="0.10"
		/>

.. dropdown:: All parameters available in ouster-os1.sensor.xml

   File: `mvsim_tutorial/definitions/ouster-os1.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/ouster-os1.sensor.xml>`_

   .. literalinclude:: ../definitions/ouster-os1.sensor.xml
      :language: xml

|


Velodyne VLP-16
##########################

.. image:: https://mrpt.github.io/imgs/mvsim-lidar-velodyne-vlp16.png
   :width: 100%
   :alt: sensor preview in MVSim

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/velodyne-vlp16.sensor"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rpm="600"
		/>

.. dropdown:: All parameters available in velodyne-vlp16.sensor.xml

   File: `mvsim_tutorial/definitions/velodyne-vlp16.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/velodyne-vlp16.sensor.xml>`_

   .. literalinclude:: ../definitions/velodyne-vlp16.sensor.xml
      :language: xml

----

RGB camera
------------------

A regular RGB (color) pin-hole camera (without lens distortion at present).
The user must provide the camera intrinsic and extrinsic parameters:

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/camera.sensor.xml"
			sensor_x="0.1" sensor_y="0.0" sensor_z="0.8"
			ncols="800"    nrows="600"
			cx="$f{800/2}" cy="$f{600/2}"
			fx="800" fy="800"
			sensor_period_sec="$f{1/20.0}"
			clip_min="0.02" clip_max="300"
			sensor_visual_scale="0.2"
		/>

.. dropdown:: All parameters available in camera.sensor.xml

   File: `mvsim_tutorial/definitions/camera.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/camera.sensor.xml>`_

   .. literalinclude:: ../definitions/camera.sensor.xml
      :language: xml

----

IMU
------------------

An inertial sensor that measures (in the current version of MVSim):

- 3D linear proper acceleration.
- 3D angular velocity.
- (Optionally) 3D orientation as a quaternion.

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/imu.sensor.xml"
			sensor_x="0.0" sensor_y="0.0" sensor_z="0.0"
			sensor_period_sec="$f{1/200.0}"
		/>

.. dropdown:: All parameters available in imu.sensor.xml

   File: `mvsim_tutorial/definitions/imu.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/imu.sensor.xml>`_

   .. literalinclude:: ../definitions/imu.sensor.xml
      :language: xml


IMU noise model
##########################

The IMU sensor implements the standard continuous-time stochastic error model
described in [Forster2016]_, with two independent noise components per axis
for both the gyroscope and accelerometer channels:

**White noise** (measurement noise).
  Zero-mean Gaussian noise added to every sample. Configured via the
  ``*_white_noise_std_noise`` parameters (standard deviation in the channel
  units: ``rad/s`` for gyroscope, ``m/s²`` for accelerometer).

**Bias random walk** (in-run stability / drift).
  A slowly-varying bias modelled as a Wiener process (integral of white
  noise). Configured via the ``*_random_walk_std_noise`` parameters
  (units: ``rad/s/√s`` for gyroscope, ``m/s²/√s`` for accelerometer).
  Set to ``0`` (default) to disable bias drift.

The discrete-time update at each simulation step of duration :math:`\Delta t` is:

.. math::

   \mathbf{b}_{k} = \mathbf{b}_{k-1}
       + \mathcal{N}\!\bigl(\mathbf{0},\;\sigma_{\mathrm{rw}}^{2}\,\Delta t\;\mathbf{I}\bigr)

.. math::

   \tilde{\mathbf{m}}_{k} = \mathbf{m}_{k}
       + \mathbf{b}_{k}
       + \mathcal{N}\!\bigl(\mathbf{0},\;\sigma_{w}^{2}\;\mathbf{I}\bigr)

The four XML parameters are:

.. list-table::
   :header-rows: 1
   :widths: 55 15 30

   * - Parameter
     - Default
     - Description
   * - ``angular_velocity_white_noise_std_noise``
     - ``2e-4``
     - Gyroscope white noise σ (rad/s)
   * - ``linear_acceleration_white_noise_std_noise``
     - ``0.017``
     - Accelerometer white noise σ (m/s²)
   * - ``angular_velocity_random_walk_std_noise``
     - ``0``
     - Gyroscope bias random walk σ (rad/s/√s)
   * - ``linear_acceleration_random_walk_std_noise``
     - ``0``
     - Accelerometer bias random walk σ (m/s²/√s)

.. note::

   The legacy parameter names ``angular_velocity_std_noise`` and
   ``linear_acceleration_std_noise`` are still accepted and map to the
   corresponding ``*_white_noise_std_noise`` parameters.

**Example** — a noisy tactical-grade IMU with bias drift:

.. code-block:: xml

     <include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/imu.sensor.xml"
       sensor_x="0.0" sensor_y="0.0" sensor_z="0.0"
       sensor_period_sec="$f{1/200.0}"
       sensor_angular_velocity_white_noise_std_noise="1.7e-4"
       sensor_linear_acceleration_white_noise_std_noise="5.88e-3"
       sensor_angular_velocity_random_walk_std_noise="1.0e-5"
       sensor_linear_acceleration_random_walk_std_noise="3.0e-4"
     />

.. [Forster2016] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza,
   "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry",
   *IEEE Transactions on Robotics*, vol. 33, no. 1, pp. 1-21, 2016.

----

2D laser scanner
------------------

.. image:: https://mrpt.github.io/imgs/mvsim-2d-lidar.png
   :width: 100%
   :alt: sensor preview in MVSim

"Classical" lidars that scan obstacles in a plane only.
These includes are available for these sensors:

Generic 2D LIDAR
##########################

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   Important parameters:
   
   - ``raytrace_3d=false`` (**DEFAULT**),  Very fast simulation using approximate 2D shapes of world elements.
   - ``raytrace_3d=true``: It uses GPU-based raytracing for exact distance calculation to world elements of arbitrary 3D shapes.

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/lidar2d.sensor.xml"
			sensor_x="0.2" sensor_y="0" sensor_z="0.50" sensor_yaw="0"
			sensor_period_sec="0.10"
			sensor_nrays="181"
			raytrace_3d="true"
			fov_degrees="270"
			sensor_name="scanner1"
		>

.. dropdown:: All parameters available in lidar2d.sensor.xml

   File: `mvsim_tutorial/definitions/lidar2d.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/lidar2d.sensor.xml>`_

   .. literalinclude:: ../definitions/lidar2d.sensor.xml
      :language: xml

|

RPLidar A2
##########################

Just like the generic Lidar above, but with a custom visualization for this particular commercial model.

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   Important parameter: See notes on ``raytrace_3d`` above.
   
   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/rplidar-a2.sensor.xml"
			sensor_x="0.2" sensor_y="0" sensor_z="0.50" sensor_yaw="0"
			sensor_period_sec="0.10"
			sensor_nrays="181"
			raytrace_3d="true"
			fov_degrees="270"
			sensor_name="scanner1"
		>

.. dropdown:: All parameters available in rplidar-a2.sensor.xml

   File: `mvsim_tutorial/definitions/rplidar-a2.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/rplidar-a2.sensor.xml>`_

   .. literalinclude:: ../definitions/rplidar-a2.sensor.xml
      :language: xml

----

Depth (RGBD) camera
---------------------

.. image:: https://mrpt.github.io/imgs/mvsim-rgbd-camera.png
   :width: 100%
   :alt: sensor preview in MVSim

An RGB+D (color plus depth) camera sensor, similar to an Intel RealSense or
ASUS Xtion / Astra. It simulates both the RGB and depth channels independently
(each with its own intrinsics and clip distances) and can optionally publish to
ROS the following topics:

.. list-table:: ROS topics published by the RGBD sensor
   :header-rows: 1
   :widths: 40 20 40

   * - Topic
     - Type
     - Notes
   * - ``<label>_image``
     - ``sensor_msgs/Image``
     - RGB image (always published when ``sense_rgb=true``)
   * - ``<label>_points``
     - ``sensor_msgs/PointCloud2``
     - XYZ pointcloud, or **XYZRGB** if ``publish_ros_colored_pointcloud=true``
   * - ``<label>_depth/image_raw``
     - ``sensor_msgs/Image``
     - 16UC1 depth image (each pixel = depth in ``rangeUnits``, default 1 mm; 0 = invalid). Published when ``publish_ros_depth_image=true``
   * - ``<label>_depth/camera_info``
     - ``sensor_msgs/CameraInfo``
     - Depth camera intrinsics. Published when ``publish_ros_depth_image=true``

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/rgbd_camera.sensor.xml"
		  sensor_x="0.2" sensor_y="0"  sensor_z="0.29"
		  sensor_period_sec="0.10"
		  show_3d_pointcloud="true"
		  publish_ros_depth_image="true"
		  publish_ros_colored_pointcloud="false"
		/>

.. dropdown:: All parameters available in rgbd_camera.sensor.xml

   File: `mvsim_tutorial/definitions/rgbd_camera.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/rgbd_camera.sensor.xml>`_

   .. literalinclude:: ../definitions/rgbd_camera.sensor.xml
      :language: xml


ROS publishing options
##########################

``publish_ros_depth_image`` (default: ``true``)
  When enabled, the sensor publishes the depth channel as a separate
  ``sensor_msgs/Image`` message encoded as **16UC1** on the topic
  ``<label>_depth/image_raw``, together with a matching
  ``sensor_msgs/CameraInfo`` on ``<label>_depth/camera_info``.
  This matches the topic layout of real RGBD cameras such as the
  ASUS Xtion / Astra (``/depth/image_raw``).

``publish_ros_colored_pointcloud`` (default: ``false``)
  When enabled **and** the sensor has ``sense_rgb=true``, the pointcloud
  published on ``<label>_points`` will include per-point RGB color
  (``XYZRGB`` fields) instead of plain ``XYZ``.


----


.. _sensors-gps:

GNSS sensor ("GPS")
---------------------
A "GPS sensor" can be attached to a robot with the code shown below. 
For it to work, the ``world`` XML needs to have a :ref:`georeference tag <world-georeference>`.

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="$(ros2 pkg prefix mvsim)/share/mvsim/definitions/gnss.sensor.xml"
		  sensor_x="0.0"  sensor_y="0.0" sensor_z="0.50"
		  sensor_period_sec="1.0"
		  sensor_name="gps"
		  sensor_horizontal_std_noise="1.5"
		  sensor_vertical_std_noise="2.5"
		  />

.. dropdown:: All parameters available in gnss.sensor.xml

   File: `mvsim_tutorial/definitions/gnss.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/definitions/gnss.sensor.xml>`_

   .. literalinclude:: ../definitions/gnss.sensor.xml
      :language: xml