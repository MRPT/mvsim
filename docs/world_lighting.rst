.. _world-lights:

Light and shadows configuration
--------------------------------------------


At present, the lighting model of ``mrpt-opengl`` defines **one ambient light** and 
**one directional light source** (i.e. placed at the infinity).

MVSim offers a number of parameters under the global ``<light> ... </light>`` tag
to tune the performance and appearance of lights and shadows (all are optional).

- ``<enable_shadows>true</enable_shadows>``. Can be used to disable casting shadows (enabled by default).
  Note that shadows may not be visible on certain ground objects, most notably, occupancy grid maps, so 
  shadows not being visible on grids is not a bug.

.. raw:: html

   <video controls autoplay loop muted> <source src="https://mrpt.github.io/mvsim-models/anims/mvsim-docs-gui-view-pointclouds.mp4" type="video/mp4"> </video>



.. code-block:: xml
   :caption: Light and shadow tuning parameters

	<mvsim_world version="1.0">
    ...
    <!-- light and shadow options -->
    <light>
      <enable_shadows>true</enable_shadows>
      <!-- <shadow_map_size>1024</shadow_map_size> -->

      <light_azimuth_deg>160.0</light_azimuth_deg>
      <light_elevation_deg>40.0</light_elevation_deg>
    </light>
    ...
	</mvsim_world>


