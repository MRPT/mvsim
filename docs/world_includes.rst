.. _world-includes:

Including other XML files
--------------------------------------------

Write me!







Limitations
=============

The use of the ``<include ...>`` XML tag is allowed **almost everywhere**, but not arbitrarily anywhere.
You can use it:

- In the main ``*.world.xml`` world definition file, anywhere as a direct child node of the top-level ``<world>`` tag.
- As a child of these XML tags:
  - ``<vehicle>``
  - ``<vehicle:class>``
  - ``<block:class>``
- As a top level entry in any other included file.

