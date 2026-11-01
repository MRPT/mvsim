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
  - ``<block>``
  - ``<block:class>``
  - ``<actor>``
  - ``<actor:class>``
- As a top level entry in any other included file.

Inside the tags above, ``<include>`` and ``<if>`` can be nested at any depth, both in
class definitions and in instances (e.g. ``<if>`` wrapping the ``<include>`` of a sensor
in a ``<vehicle>``). The ``<for>`` tag is only available in the world file itself.

