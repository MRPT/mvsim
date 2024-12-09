.. _world-georeference:

Georeference options
--------------------------------------------

For :ref:`GNSS sensors <sensors-gps>` ("GPS") to work, they need to know the transformation between
the simulator world local coordinates and the Earth.
This can be specified with a ``<georeference>`` tag as documented below.


.. code-block:: xml
   :caption: Georeference example #1: Using geodetic coordinates

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

.. code-block:: xml
   :caption: Georeference example #2: Using UTM world coordinates

    <mvsim_world version="1.0">
    ...
      <!-- Define georeferenced coordinates to the world so GNSS/GPS sensors can be properly simulated -->
      <georeference>
        <world_is_utm>true</world_is_utm>
        <!-- A reference point roughly within the area of the map to linearize coordinates -->
        <latitude>36.894718</latitude>
        <longitude>-2.316988</longitude>
        <height>100.0</height>
      </georeference>
    ...
    </mvsim_world>


Parameters for worlds in local coordinates and GNSS geodetic reference:

- ``<latitude>yy</latitude>`` and ``<longitude>xx</longitude>``. Geodetic coordinates of the world (0,0,0) origin.

- ``<height>zz</height>``: The height over the WGS84 ellipsoid for the world (0,0,0).

- ``<world_to_enu_rotation_deg>tt</world_to_enu_rotation_deg>``: An optional rotation (in degrees) if you want North not to be aligned with +Y as it is the default. Does not apply when using UTM coordinates.


Parameters for worlds with local coordinates as `UTM coordinates <https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system>`_:

- ``<world_is_utm>true</world_is_utm>``: Indicates that world coordinates are UTM coordinates, defined in the zone of the reference (lat,lon) geodetic coordinates.
