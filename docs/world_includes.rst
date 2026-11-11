.. _world-includes:

Including other XML files
--------------------------------------------

The ``<include file="..."/>`` tag inserts the contents of another XML file, with the
remaining attributes of the tag defined as variables for that file (e.g. ``${my_var|default}``).
Relative paths inside the included file are relative to its own directory.

A complete world file (with its ``<mvsim_world>`` root tag) can be included too: its
contents are added to the including world. This allows, for example, adding robots to a scene
world that can also be loaded by itself:

.. code-block:: xml

    <mvsim_world version="1.0">
      <include file="scenes/my_field.world.xml"/>
      <include file="robots/my_robot.vehicle.xml" use_camera="true"/>
      <vehicle name="robot1" class="my_robot">
        <init_pose>5 2 90</init_pose>
      </vehicle>
    </mvsim_world>


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

