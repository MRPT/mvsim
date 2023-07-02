.. _world_lighting:

Light and shadows configuration
--------------------------------------------


At present, the lighting model of ``mrpt-opengl`` defines
**one directional light source** (i.e. placed at the infinity),
with one color and three component intensities (ambient, diffuse, and specular).
See `mrpt::opengl::TLightParameters <https://docs.mrpt.org/reference/latest/struct_mrpt_opengl_TLightParameters.html>`_
and `mrpt-opengl <https://docs.mrpt.org/reference/latest/group_mrpt_opengl_grp.html>`_ for further details.

MVSim offers a number of parameters under the global ``<light> ... </light>`` tag
to tune the performance and appearance of lights and shadows (all are optional).

Light control
================

- ``<light_color>#ffffff</light_color>``: The light color (see formatting for :ref:`%color <world_value_parsing>`).

- ``<light_ambient>0.5</light_ambient>``: Ambient component of the light (0 to 1).

- ``<light_azimuth>45.0</light_azimuth>`` and ``<light_elevation>70.0</light_elevation>``: 
  The orbit-like azimuth and elevation angles (in **degrees**) of the directional light source.
  For example, an elevation of ``90`` means a pure vertical (downwards) light.

.. raw:: html

   <video controls autoplay loop muted> <source src="https://mrpt.github.io/mvsim-models/anims/mvsim-docs-light-direction.mp4" type="video/mp4"> </video>



Shadows control
================

- ``<enable_shadows>true</enable_shadows>``. Can be used to disable casting shadows (enabled by default).
  Note that shadows may not be visible on certain ground objects, most notably, occupancy grid maps, so 
  shadows not being visible on grids is not a bug.

.. raw:: html

   <video controls autoplay loop muted> <source src="https://mrpt.github.io/mvsim-models/anims/mvsim-docs-shadows-on-off.mp4" type="video/mp4"> </video>

|


- ``<shadow_map_size>2048</shadow_map_size>``. The resolution of the `shadow map texture <https://en.wikipedia.org/wiki/Shadow_mapping>`_ in pixels.
  Larger values will give more well-defined shadow borders, at the cost of higher GPU usage. Smaller values may slightly improve speed.

.. list-table:: 

    * - .. figure:: https://mrpt.github.io/mvsim-models/screenshots/shadow_map_size-512.png

           shadow_map_size=512

      - .. figure:: https://mrpt.github.io/mvsim-models/screenshots/shadow_map_size-2048.png

           shadow_map_size=2048 (Default)


|


- ``<light_clip_plane_min>0.1</light_clip_plane_min>`` and ``<light_clip_plane_max>900</light_clip_plane_max>``.
  The unidirectional light source is not actually placed at the infinity, but at a **large-enough distance**, defined by the ``light_clip_plane_max``
  parameter (in meters), measured from the camera look-at point. There is also a minimum distance from the light source from which to start computing
  shadows, defined by ``light_clip_plane_min``. Ideally, the smaller the range between these two numbers, the more accurate shadows will be.


- ``<shadow_bias>1e-5</shadow_bias>``, ``<shadow_bias_cam2frag>1e-5</shadow_bias_cam2frag>``, and 
  ``<shadow_bias_normal>1e-4</shadow_bias_normal>`` are all ``mrpt-opengl`` heuristic parameters to 
  fight the "Peter-Panning" and "shadow acne" artifacts in shadow casting [Microsoft_Shadows]_.

- ``<eye_distance_to_shadow_map_extension>2.0</eye_distance_to_shadow_map_extension>``: This unitless (meter/meter)
  scale factor determines the extension (in meters) of the light shadow-generation square fructum.

- ``<minimum_shadow_map_extension_ratio>0.005</minimum_shadow_map_extension_ratio>``: Unitless (meter/meter)
  ratio of ``light_clip_plane_max`` that determines the minimum size of the shadow map area. Larger values
  are required for larger contiguous volumes of the world are visible at once, e.g. larger rooms.


|


.. code-block:: xml
   :caption: Light and shadow tuning parameters: example

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


.. [Microsoft_Shadows] Common Techniques to Improve Shadow Depth Maps, Microsoft, https://learn.microsoft.com/en-us/windows/win32/dxtecharts/common-techniques-to-improve-shadow-depth-maps
