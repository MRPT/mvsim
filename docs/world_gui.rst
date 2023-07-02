GUI and visualization options
--------------------------------------------

Available parameters under the global ``<gui> ... </gui>`` tag (all are optional).

- ``<win_w>800</win_w>`` and ``<win_h>600</win_h>``. Size of the main GUI window, in pixels.

- ``<start_maximized>true</start_maximized>``. Whether to start maximized.

- ``<refresh_fps>20</refresh_fps>``. GUI refresh rate (in FPS or Hz). Faster will
  lead to smoother graphics and animations of the vehicle motion, at a higher CPU/GPU
  cost. Publication of sensor readings, odometry, etc. is not affected at all by this parameter.

- ``<show_sensor_points>true</show_sensor_points>``. If enabled (default),
  obstacle readings (RGBD cameras, 2D or 3D Lidars, etc.) will be visible as point clouds.
  This option can be also dynamically switched from the UI:

.. raw:: html

   <video controls autoplay loop muted> <source src="https://mrpt.github.io/mvsim-models/anims/mvsim-docs-gui-view-pointclouds.mp4" type="video/mp4"> </video>

- ``<headless>false</headless>``. If enabled, MVSim will run without
  any interactive GUI. Useful to save some CPU/GPU usage or to run 
  MVSim inside docker containers.

- ``<cam_point_to>0  0  0</cam_point_to>``. Defines the (x,y,z) coordinates
  of the point the camera should look at upon start.

- ``<show_forces>false</show_forces>`` and ``<force_scale>0.01</force_scale>``. If
  ``show_forces`` is set to ``true``, reaction forces will be visible as thin lines, using
  the given scale in meters/Newton. This option can be also dynamically switched from the UI:

.. raw:: html

   <video controls autoplay loop muted> <source src="https://mrpt.github.io/mvsim-models/anims/mvsim-docs-gui-view-forces.mp4" type="video/mp4"> </video>


- ``<ortho>false</ortho>``. If set to ``true``, the GUI will show an orthographic view 
  of the world. Otherwise, it will use a standard perspective camera.

- ``<fov_deg>60.0</fov_deg>``. If ``ortho`` is ``false``, this defines
  the GUI camera vertical field of view, in degrees.

- ``<cam_distance>80.0</cam_distance>``. The initial distance from the camera
  point-to location and the camera focal point.

- ``<cam_azimuth>45.0</cam_azimuth>`` and ``<cam_elevation>40.0</cam_elevation>``. 
  The initial elevation and azimuth of the orbit model to define
  the camera placement and pose.

- ``<follow_vehicle>r1</follow_vehicle>``. If not empty, the GUI camera will accept rotations made
  by the user, but it will always exactly follow the robot given by its ``name``.

- ``<clip_plane_min>0.05</clip_plane_min>`` and ``<clip_plane_max>10e3</clip_plane_max>``. 
  The shortest and farthest distances that are visible in the GUI
  camera `view frustrum <https://en.wikipedia.org/wiki/Viewing_frustum>`_.


.. code-block:: xml
   :caption: GUI and visualization options

	<mvsim_world version="1.0">
	...
		<!-- GUI options -->
		<gui>
			<!-- Is camera orthographic or projective? -->
			<ortho>false</ortho>

			<!-- Show reaction forces on wheels with lines -->
			<show_forces>false</show_forces>

			<!-- default camera distance in world units -->
			<cam_distance>35</cam_distance>

			<!-- camera vertical field of view in degrees -->
			<fov_deg>60</fov_deg>

			<!-- <follow_vehicle>r1</follow_vehicle> -->
		</gui>
	...
	</mvsim_world>


