Top-level and global simulation settings
--------------------------------------------

The entire world definition file must be contained within a top-level
tag ``<mvsim_world>``. At present, a world-definition version number may
optionally be added as an attribute, with ``1.0`` the current version.

Available parameters (all are optional):

- ``<simul_timestep>0</simul_timestep>``. The fixed timestep in seconds
  for numerical integration of the physics engine. Note that if ``T`` is larger than
  the smallest period of a given sensor, some sensor readings will be missed. That is
  why there is a "magic" value of ``T`` of ``0`` (the default) which automatically
  analyze all sensors and sets this timestep accordingly to ensure no reading is lost.

- ``<gravity>9.81</gravity>``. Gravity acceleration (m/s²), in the negative Z axis 
  direction (i.e. downwards).

- ``<joystick_enabled>false</joystick_enabled>``. If enabled, a gamepad or joystick 
  can be used to control the selected vehicle. (Default is ``false`` hence no responding to gamepads).

- ``<server_address>localhost</server_address>``. The IP address the server
  must be bound to. Refer to :ref:`architecture`.

- ``<b2d_vel_iters>8</b2d_vel_iters>`` and ``<b2d_pos_iters>3</b2d_pos_iters>``. 
  Velocity and position iteration count (refer to libbox2d docs).

- ``<collision_threshold>0.03</collision_threshold>``: If the distance between the closest points of 
  a contact manifold (see box2d docs) for two bodies is below this distance, it is marked as "in collision",
  and reported as such by the collision API.

- ``<save_to_rawlog>my_dataset.rawlog</save_to_rawlog>``: If present, all sensor observations
  will be saved into an MRPT dataset in ``.rawlog`` format. One file will be created per vehicle,
  by adding the vehicle name to the provided file name.

- ``<rawlog_odometry_rate>10.0</rawlog_odometry_rate>``: If ``save_to_rawlog`` is enabled,
  this parameter defines the rate (in Hz) to generate (wheels) odometry observations (Default is 10 Hz).

- ``<save_ground_truth_trajectory>gt.txt</save_ground_truth_trajectory>``: If non-empty, the
  ground truth trajectory of all vehicles will be saved to a text file in the 
  `TUM trajectory format <https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format>`_.
  One file will be created per vehicle,  by adding the vehicle name to the provided file name.

- ``<ground_truth_rate>50.0</ground_truth_rate>``: If ``save_ground_truth_trajectory`` is enabled,
  this parameter defines the rate (in Hz) to generate entries in the trajectory file (Default is 50 Hz).

- ``<random_seed>42</random_seed>``: Makes the simulation reproducible. The noise of every
  sensor (2D and 3D LiDARs, IMU, GNSS, depth cameras) is drawn from its own generator,
  seeded from this value and the vehicle and sensor names, so it does not depend on the
  number of threads or on the other sensors. The generator used by random expressions in
  the world file (``$f{rand()}``, ``$f{randn()}``, ``$f{unifrnd(a,b)}``) is also seeded
  when this tag is parsed, so it must appear *before* the elements that use them.
  It accepts variables, e.g. ``<random_seed>$env{SEED}</random_seed>``; a negative
  value (or no tag) keeps non-deterministic random numbers. Put it once, at the top level
  and at the beginning of the file: the vehicle odometry noise is also drawn from that
  generator when the vehicles are created. RGB camera pixel noise is seeded per sensor too
  (without ``<random_seed>``, it uses a fixed seed, as before, so it is the same in every run).

.. code-block:: xml
   :caption: Top-level and global settings example

	<mvsim_world version="1.0">
		<!-- General simulation options -->
		<simul_timestep>0</simul_timestep> <!-- Simulation fixed-time interval for numerical integration [s], or 0 to auto-determine -->

		<!-- Normally disabled: If this tag is present, *all* sensor data is also stored
		 into an MRPT rawlog file. Useful for collecting datasets without ROS. -->
		<!-- <save_to_rawlog>sensor_dataset.rawlog</save_to_rawlog> -->
		...
	</mvsim_world>


