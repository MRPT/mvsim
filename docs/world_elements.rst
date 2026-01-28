.. _world_elements:

==============
World Elements
==============

World elements define the static and dynamic components of the simulation environment in MVSim.
These elements provide visual references, terrain data, collision geometry, and logical regions
for robot simulation scenarios.


.. contents::
   :depth: 1
   :local:
   :backlinks: none


Occupancy Grid Map
~~~~~~~~~~~~~~~~~~

**<element class="occupancy_grid">** represents an MRPT occupancy map that defines occupied
and free space in the simulation world. This element supports collision detection with robots
and other dynamic objects.

The map can be specified using:

* A black-and-white image file (where black represents occupied cells)
* An MRPT grid map file format

**XML Configuration:**

.. code-block:: xml

   <element class="occupancy_grid">
       <file>path/to/map_image.png</file>
   </element>

**Subtags:**

* **<file>** - Specifies the file path to the image or MRPT map file


-------

|

Ground Grid
~~~~~~~~~~~

**<element class="ground_grid">** displays a metric grid overlay for visual reference in the
simulation. This element has no physical properties and serves purely for visualization purposes.

**XML Configuration:**

.. code-block:: xml

   <element class="ground_grid">
       <!-- Optional: make grid float and follow a vehicle -->
       <floating>true</floating>
       <float_center_at_vehicle_name>r1</float_center_at_vehicle_name>
   </element>

-------

|

Horizontal Plane
~~~~~~~~~~~~~~~~

**<element class="horizontal_plane">** defines a flat horizontal surface that can serve as
ground floor or ceiling. Supports textures, collision detection, and shadow rendering.

**XML Configuration:**

.. code-block:: xml

   <element class="horizontal_plane">
       <cull_face>BACK</cull_face>
       <x_min>-20</x_min>
       <y_min>-20</y_min>
       <x_max>20</x_max>
       <y_max>20</y_max>
       <z>0.0</z>
       <texture>path/to/texture.png</texture>
       <texture_size_x>2.0</texture_size_x>
       <texture_size_y>2.0</texture_size_y>
   </element>

**Subtags:**

* **<x_min>, <x_max>** - Minimum and maximum X coordinates (world units)
* **<y_min>, <y_max>** - Minimum and maximum Y coordinates (world units)
* **<z>** - Height of the plane (world units)
* **<cull_face>** - Face culling mode: NONE, FRONT, or BACK
* **<texture>** - Optional texture image path (local or remote URL)
* **<texture_size_x>, <texture_size_y>** - Texture tiling size in world units

-------

|

Vertical Plane
~~~~~~~~~~~~~~

**<element class="vertical_plane">** creates a vertical wall defined by two endpoints. Useful
for perimeter walls, barriers, and indoor partitions.

**XML Configuration:**

.. code-block:: xml

   <element class="vertical_plane">
       <cull_face>FRONT</cull_face>
       <x0>-15</x0>
       <y0>13</y0>
       <x1>-15</x1>
       <y1>-13</y1>
       <z>0.0</z>
       <height>3.0</height>
       <texture>wall_texture.png</texture>
       <texture_size_x>3.0</texture_size_x>
       <texture_size_y>3.0</texture_size_y>
   </element>

**Subtags:**

* **<x0>, <y0>** - Starting point coordinates
* **<x1>, <y1>** - Ending point coordinates
* **<z>** - Base height of the wall
* **<height>** - Wall height above the base
* **<cull_face>** - Face culling mode: NONE, FRONT, or BACK
* **<texture>** - Optional texture image path
* **<texture_size_x>, <texture_size_y>** - Texture tiling size

-------

|

Elevation Map
~~~~~~~~~~~~~

**<element class="elevation_map">** creates a Digital Elevation Model (DEM) representing
terrain with varying heights. The mesh-based map is built from an elevation bitmap where
lighter pixels represent higher elevations and darker pixels represent lower elevations.


**XML Configuration:**

.. code-block:: xml

   <element class="elevation_map">
       <resolution>4.0</resolution>
       <elevation_image>path/to/elevation.png</elevation_image>
       <elevation_image_min_z>0.0</elevation_image_min_z>
       <elevation_image_max_z>10.0</elevation_image_max_z>
       <texture_image>path/to/texture.png</texture_image>
       <mesh_color>#808080</mesh_color>
       <corner_min_x>-24</corner_min_x>
       <corner_min_y>-2</corner_min_y>
       <texture_extension_x>8.0</texture_extension_x>
       <texture_extension_y>4.0</texture_extension_y>
       <model_split_size>3.0</model_split_size>
   </element>

**Subtags:**

* **<elevation_image>** - Path to the elevation bitmap (grayscale image)
* **<elevation_data_matrix>** - Alternative to image: inline elevation matrix data
* **<elevation_image_min_z>** - Minimum height in world units (darkest pixels)
* **<elevation_image_max_z>** - Maximum height in world units (lightest pixels)
* **<texture_image>** - Path to texture image for the elevation mesh
* **<mesh_color>** - Mesh color in HEX RGB format (e.g., #FF0000)
* **<resolution>** - Mesh XY resolution/scale in world units
* **<corner_min_x>, <corner_min_y>** - World coordinates of the map corner
* **<texture_extension_x>, <texture_extension_y>** - Texture size in world units (0=auto)
* **<texture_image_rotate>** - Rotation angle for texture in degrees
* **<model_split_size>** - Split mesh into smaller chunks for correct rendering of transparent objects

.. warning::
   Do not use both **<texture_image>** and **<mesh_color>** simultaneously. Choose one method
   for coloring the elevation mesh.

-------

|

Point Cloud
~~~~~~~~~~~

**<element class="point_cloud">** allows simulating a world defined by a 3D point cloud.
Useful for loading scanned environments or LiDAR data.

**XML Configuration:**

.. code-block:: xml

   <element class="point_cloud">
       <file>path/to/pointcloud.pcd</file>
       <render_points_size>3.0</render_points_size>
   </element>

-------

|

Property Region
~~~~~~~~~~~~~~~

**<element class="property_region">** defines a non-visual world element that assigns named
properties within a 3D axis-aligned bounding box. This is useful for simulating GPS-denied
areas, zones with different friction coefficients, or other logical regions that affect
robot behavior.

Properties can have any of these types: ``bool``, ``string``, or ``double``.

**XML Configuration:**

.. code-block:: xml

   <element class="property_region">
       <property_name>gps_no_coverage</property_name>
       <value_bool>true</value_bool>
       <x_min>-5.0</x_min>
       <x_max>5.0</x_max>
       <y_min>10.0</y_min>
       <y_max>20.0</y_max>
       <!-- z limits default to [-1000, 1000] if not specified -->
       <z_min>-10.0</z_min>
       <z_max>10.0</z_max>
   </element>

**Subtags:**

* **<property_name>** - Name identifier for the property
* **<value_bool>** - Boolean property value
* **<value_string>** - String property value
* **<value_double>** - Double precision floating point value
* **<x_min>, <x_max>** - X-axis bounds of the region
* **<y_min>, <y_max>** - Y-axis bounds of the region
* **<z_min>, <z_max>** - Z-axis bounds (default: [-1000, 1000])

**Well-Defined Properties:**

The following property names have special meanings in the simulator:

* **gps_no_coverage** (bool) - Defines zones without GPS coverage where GNSS sensors will not function

**Example - GPS-Denied Zone:**

.. code-block:: xml

   <element class="property_region">
       <property_name>gps_no_coverage</property_name>
       <value_bool>true</value_bool>
       <x_min>-5.0</x_min>
       <x_max>5.0</x_max>
       <y_min>10.0</y_min>
       <y_max>20.0</y_max>
   </element>

-------

|

SkyBox
~~~~~~

**<element class="skybox">** adds a skybox visual decoration to the simulation environment.
Provides an immersive background using six texture images representing the sky in all directions.

.. note::
   This element requires MVSim to be built against MRPT >= 2.7.0

**XML Configuration:**

.. code-block:: xml

   <element class="skybox">
       <textures>https://example.com/skybox_%s.jpg</textures>
   </element>

**Subtags:**

* **<textures>** - URI pattern with ``%s`` placeholder that will be replaced with:
  "Up", "Right", "Left", "Front", "Down", "Back" to locate the six texture files

