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

- ``<server_address>localhost</server_address>``. The IP address the server
  must be bound to. Refer to :ref:`architecture`.

- ``<b2d_vel_iters>8</b2d_vel_iters>`` and ``<b2d_pos_iters>3</b2d_pos_iters>``. 
  Velocity and position iteration count (refer to libbox2d docs).


.. code-block:: xml
   :caption: Top-level and global settings example

	<mvsim_world version="1.0">
		<!-- General simulation options -->
		<simul_timestep>0</simul_timestep> <!-- Simulation fixed-time interval for numerical integration [s], or 0 to auto-determine -->
		...
	</mvsim_world>

