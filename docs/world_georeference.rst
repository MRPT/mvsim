.. _world-georeference:

Georeference options
--------------------------------------------

For GNSS ("GPS") sensors to work, they need to know the transformation between
the simulator world local coordinates and the Earth.
This can be specified with a ``<georeference>`` tag as documented below.


.. code-block:: xml
   :caption: Georeference example (Using geodetic coordinates)

    <mvsim_world version="1.0">
    ...
      <!-- Define georeferenced coordinates to the world so GNSS/GPS sensors can be properly simulated -->
      <georeference>
        <latitude>36.894718</latitude>
        <longitude>-2.316988</longitude>
        <height>100.0</height>
        <world_to_enu_rotation_deg>0.0</world_to_enu_rotation_deg>
      </georeference>
    ...
    </mvsim_world>


- ``<latitude>yy</latitude>`` and ``<longitude>xx</longitude>``. Geodetic coordinates of the world (0,0,0) origin.

- ``<height>zz</height>``: The height over the WGS84 ellipsoid for the world (0,0,0).

- ``<world_to_enu_rotation_deg>tt</world_to_enu_rotation_deg>``: An optional rotation (in degrees) if you want North not to be aligned with +Y as it is the default.
