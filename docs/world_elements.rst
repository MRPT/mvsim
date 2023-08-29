.. _world_elements:

World elements
--------------------------------------------

**<element class="occupancy\_grid">** depicts MRPT occupancy map which
can be specified with both image file (black and while) and MRPT grid
maps. **<file>** specifies file path to image of the map.

**<element class="ground\_grid">** is the metric grid for visual
reference.

**<element class="elevation\_map">** is an elevation map
(!experimental). Mesh-based map is build of elevation map in simple
bitmap where whiter means higher and darker - lower.

This tag has several subtags:

-  **<elevation\_image>** - path the elevation bitmap itself

-  **<elevation\_image\_min\_z>** - minimum height in world units

-  **<elevation\_image\_max\_z>** - maximum height in world units

-  **<texture\_image>** - path texture image for elevation bitmap. Mus
   not be used with *mesh\_color* simultaneously

-  **<mesh\_color>** - mesh color in HEX RGB format

-  **<resolution>** - mesh XY scale

