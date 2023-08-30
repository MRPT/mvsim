Sensors
===================

This page describes the predefined sensors and their parameters.

.. contents:: List of sensors
   :depth: 1
   :local:
   :backlinks: none

3D LIDAR
------------------

Due to the very specific details of LIDAR models which needs to be accounted for
an accurate simulation of real commercial devices, MVSim provides ready-to-use
XML "includes" for some common models:

HELIOS 32 (26 deg FOV)
##########################

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="definitions/helios-32-FOV-26.sensor.xml"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rate="10.0"
		/>

.. dropdown:: Include file helios-32-FOV-26.sensor.xml

   File: `mvsim_tutorial/definitions/helios-32-FOV-26.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/definitions/helios-32-FOV-26.sensor.xml>`_

   .. literalinclude:: ../mvsim_tutorial/definitions/helios-32-FOV-26.sensor.xml
      :language: xml


HELIOS 32 (31 deg FOV)
##########################

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="definitions/helios-32-FOV-31.sensor.xml"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rate="10.0"
		/>


.. dropdown:: Include file helios-32-FOV-31.sensor.xml

   File: `mvsim_tutorial/definitions/helios-32-FOV-31.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/definitions/helios-32-FOV-31.sensor.xml>`_

   .. literalinclude:: ../mvsim_tutorial/definitions/helios-32-FOV-31.sensor.xml
      :language: xml


HELIOS 32 (70 deg FOV)
##########################

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="definitions/helios-32-FOV-70.sensor.xml"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rate="10.0"
		/>

.. dropdown:: Include file helios-32-FOV-70.sensor.xml

   File: `mvsim_tutorial/definitions/helios-32-FOV-70.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/definitions/helios-32-FOV-70.sensor.xml>`_

   .. literalinclude:: ../mvsim_tutorial/definitions/helios-32-FOV-70.sensor.xml
      :language: xml


OUSTER OS1
##########################

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="definitions/ouster-os1.sensor"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_period_sec="0.10"
		/>

.. dropdown:: Include file ouster-os1.sensor.xml

   File: `mvsim_tutorial/definitions/ouster-os1.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/definitions/ouster-os1.sensor.xml>`_

   .. literalinclude:: ../mvsim_tutorial/definitions/ouster-os1.sensor.xml
      :language: xml



Velodyne VLP-16
##########################

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="definitions/velodyne-vlp16.sensor"
		  sensor_x="0.10" sensor_z="0.30"
		  sensor_std_noise="0.005"
		  sensor_name="lidar1"
		  sensor_rpm="600"
		/>

.. dropdown:: Include file velodyne-vlp16.sensor.xml

   File: `mvsim_tutorial/definitions/velodyne-vlp16.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/definitions/velodyne-vlp16.sensor.xml>`_

   .. literalinclude:: ../mvsim_tutorial/definitions/velodyne-vlp16.sensor.xml
      :language: xml


RGB camera
------------------

A regular RGB (color) pin-hole camera (without lens distortion at present).
The user must provide the camera intrinsic and extrinsic parameters:

.. dropdown:: To use in your robot, copy and paste this inside a ``<vehicle>`` or ``<vehicle:class>`` tag.
   :open:

   .. code-block:: xml

		<include file="definitions/camera.sensor.xml"
			sensor_x="0.1" sensor_y="0.0" sensor_z="0.8"
			ncols="800"    nrows="600"
			cx="$f{800/2}" cy="$f{600/2}"
			fx="800" fy="800"
			sensor_period_sec="$f{1/20.0}"
			clip_min="0.02" clip_max="300"
			sensor_visual_scale="0.2"
		/>

.. dropdown:: Include file camera.sensor.xml

   File: `mvsim_tutorial/definitions/camera.sensor.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/definitions/camera.sensor.xml>`_

   .. literalinclude:: ../mvsim_tutorial/definitions/camera.sensor.xml
      :language: xml


IMU
------------------

Write me!


2D laser scanner
------------------

Write me!


Depth (RGBD) camera
---------------------

Write me!


Generic definitions for all sensors
-----------------------------------------

Sensors are defined with **<sensor>** tag. It has attributes *type* and
*name*.

2D Lidar sensor: its type is *laser*. Subtags are:

-  **<pose>** - an MRPT CPose3D string value

-  **<fov\_degrees>** - FOV of the laser scanner

-  **<sensor\_period>** - period in seconds when sensor sends updates

-  **<nrays>** - laser scanner rays per FOV

-  **<range\_std\_noise>** - standard deviation of noise in distance
   measurements

-  **<angle\_std\_noise\_deg>** - standatd deviation of noise in angles
   of rays

-  **<bodies\_visible>** - boolean flag to see other robots or not


