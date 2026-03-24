.. _world_lighting:

Light and shadows configuration
--------------------------------------------


The lighting model uses MRPT's multi-light rendering pipeline with
**hemisphere ambient lighting**, a **directional light source** (placed at infinity)
that casts shadows, and optional **point** and **spot** light sources.
See `mrpt::viz::TLightParameters <https://docs.mrpt.org/reference/latest/struct_mrpt_opengl_TLightParameters.html>`_
and `mrpt-opengl <https://docs.mrpt.org/reference/latest/group_mrpt_opengl_grp.html>`_ for further details.

MVSim offers a number of parameters under the global ``<lights> ... </lights>`` tag
to tune the performance and appearance of lights and shadows (all are optional).

Directional light
===================

The primary directional light (sun-like, infinitely far away, parallel rays) is always present.
Its direction is controlled via azimuth and elevation angles, and it is the only light that casts shadows.

- ``<light_color>#ffffff</light_color>``: The light color (see formatting for :ref:`%color <world_value_parsing>`).

- ``<light_diffuse>0.8</light_diffuse>``: Diffuse intensity of the directional light (0 to 1).

- ``<light_specular>0.6</light_specular>``: Specular intensity of the directional light (0 to 1).

- ``<light_azimuth_deg>45.0</light_azimuth_deg>`` and ``<light_elevation_deg>70.0</light_elevation_deg>``:
  The orbit-like azimuth and elevation angles (in **degrees**) of the directional light source.
  For example, an elevation of ``90`` means a pure vertical (downwards) light.

.. raw:: html

   <video controls autoplay loop muted> <source src="https://mrpt.github.io/mvsim-models/anims/mvsim-docs-light-direction.mp4" type="video/mp4"> </video>


Ambient lighting
=================

MVSim uses **hemisphere ambient lighting**: surfaces facing up receive the sky ambient color,
surfaces facing down receive the ground ambient color, with smooth interpolation for
surfaces at intermediate angles. This produces a much more natural look than flat ambient,
especially for outdoor scenes.

- ``<light_ambient>0.4</light_ambient>``: Overall ambient intensity scale (0 to 1).

- ``<ambient_sky_color>#e0e8ff</ambient_sky_color>``: The ambient color for upward-facing surfaces.
  Default is a slight blue tint (``#e0e8ff``), simulating sky light.

- ``<ambient_ground_color>#403a30</ambient_ground_color>``: The ambient color for downward-facing surfaces.
  Default is a dark warm brown (``#403a30``), simulating ground-bounce light.

To get flat (non-hemisphere) ambient lighting, set both colors to the same value (e.g. ``#ffffff``).


Point lights
=============

Point lights emit in all directions from a position in world coordinates.
They do **not** cast shadows. Multiple point lights can be defined by adding
``<point_light>`` child elements inside ``<lights>``.

Each ``<point_light>`` supports the following child tags (all optional, shown with defaults):

- ``<position>X Y Z</position>``: Position in world coordinates (default: ``0 0 3``).
- ``<color>#ffffff</color>``: Light color in ``#RRGGBB[AA]`` format (default: white).
- ``<diffuse>0.8</diffuse>``: Diffuse intensity (0 to 1).
- ``<specular>0.5</specular>``: Specular intensity (0 to 1).
- ``<attenuation_constant>1.0</attenuation_constant>``: Constant attenuation factor.
- ``<attenuation_linear>0.09</attenuation_linear>``: Linear attenuation factor.
- ``<attenuation_quadratic>0.032</attenuation_quadratic>``: Quadratic attenuation factor.

The light intensity at distance *d* is: ``1 / (constant + linear*d + quadratic*d²)``.


Spot lights
============

Spot lights emit in a cone from a position along a direction.
They do **not** cast shadows. Multiple spot lights can be defined by adding
``<spot_light>`` child elements inside ``<lights>``.

Each ``<spot_light>`` supports the following child tags (all optional, shown with defaults):

- ``<position>X Y Z</position>``: Position in world coordinates (default: ``0 0 3``).
- ``<direction>X Y Z</direction>``: Direction vector the spot points towards (default: ``0 0 -1``).
- ``<color>#ffffff</color>``: Light color in ``#RRGGBB[AA]`` format (default: white).
- ``<diffuse>0.8</diffuse>``: Diffuse intensity (0 to 1).
- ``<specular>0.5</specular>``: Specular intensity (0 to 1).
- ``<inner_cutoff_deg>12.5</inner_cutoff_deg>``: Inner cone half-angle in degrees (full intensity).
- ``<outer_cutoff_deg>17.5</outer_cutoff_deg>``: Outer cone half-angle in degrees (light fades to zero).
- ``<attenuation_constant>1.0</attenuation_constant>``: Constant attenuation factor.
- ``<attenuation_linear>0.09</attenuation_linear>``: Linear attenuation factor.
- ``<attenuation_quadratic>0.032</attenuation_quadratic>``: Quadratic attenuation factor.

.. note::

   Up to 8 simultaneous light sources are supported (including the primary directional light).
   This limit is defined by the shader pipeline.


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
   :caption: Complete lighting configuration example

   <mvsim_world version="1.0">
    ...
    <lights>
      <!-- Primary directional light (sun) -->
      <light_azimuth_deg>160.0</light_azimuth_deg>
      <light_elevation_deg>40.0</light_elevation_deg>
      <light_color>#ffffff</light_color>
      <light_diffuse>0.8</light_diffuse>
      <light_specular>0.6</light_specular>

      <!-- Hemisphere ambient lighting -->
      <light_ambient>0.4</light_ambient>
      <ambient_sky_color>#e0e8ff</ambient_sky_color>
      <ambient_ground_color>#403a30</ambient_ground_color>

      <!-- Shadow settings -->
      <enable_shadows>true</enable_shadows>
      <!-- <shadow_map_size>4096</shadow_map_size> -->

      <!-- Point light: overhead lamp -->
      <point_light>
        <position>5.0 3.0 4.0</position>
        <color>#ffe0a0</color>
        <diffuse>0.6</diffuse>
        <specular>0.3</specular>
        <attenuation_quadratic>0.05</attenuation_quadratic>
      </point_light>

      <!-- Spot light: ceiling spotlight -->
      <spot_light>
        <position>0.0 0.0 5.0</position>
        <direction>0.0 0.0 -1.0</direction>
        <color>#ffffff</color>
        <diffuse>0.7</diffuse>
        <inner_cutoff_deg>15.0</inner_cutoff_deg>
        <outer_cutoff_deg>25.0</outer_cutoff_deg>
      </spot_light>
    </lights>
    ...
   </mvsim_world>


.. [Microsoft_Shadows] Common Techniques to Improve Shadow Depth Maps, Microsoft, https://learn.microsoft.com/en-us/windows/win32/dxtecharts/common-techniques-to-improve-shadow-depth-maps
