.. _world_blocks:

Definition of blocks
--------------------------------------------

Blocks are non-vehicle objects in the simulation world, typically representing obstacles, 
furniture, buildings, or any static or dynamic object that is not a controllable robot/vehicle.

An XML block with tag name ``<block:class>`` *can* be provided for each
block *class*, then each ``<block>`` will *instantiate* a block of a particular *class*.

.. note:: 
   Blocks and vehicles share the common :ref:`Simulable <world_simulable>` and 
   :ref:`VisualObject <world_visual_object>` interfaces.


Block classes
=======================

Block classes are reusable templates defined with ``<block:class>`` tags. They allow you to
define a block type once and instantiate it multiple times with different poses or parameters.

.. code-block:: xml

    <block:class name="wooden_box">
        <mass>50.0</mass>
        <zmin>0.0</zmin>
        <zmax>1.0</zmax>
        <color>#8B4513</color>
        <ground_friction>0.8</ground_friction>
        
        <shape>
            <pt>-0.5 -0.5</pt>
            <pt>-0.5  0.5</pt>
            <pt> 0.5  0.5</pt>
            <pt> 0.5 -0.5</pt>
        </shape>
        
        <visual>
            <model_uri>https://example.com/models/box.dae</model_uri>
            <model_scale>1.0</model_scale>
        </visual>
    </block:class>


Block parameters
=======================

Blocks support the following parameters, which can be defined either in the ``<block:class>``
definition or in individual ``<block>`` instances:

Physical properties
^^^^^^^^^^^^^^^^^^^^^

- **mass**: Mass of the block in kilograms (Default: 30.0)
- **static**: Boolean indicating if the block is fixed in place (Default: false)
- **ground_friction**: Friction coefficient with the ground (Default: 0.5)
- **lateral_friction**: Lateral friction coefficient (Default: 0.5)
- **restitution**: Coefficient of restitution for collisions (Default: 0.01)
- **intangible**: If true, the block is rendered but doesn't collide or affect sensors (Default: false)

Geometry properties
^^^^^^^^^^^^^^^^^^^^^

- **zmin**: Distance from bottom of the block to ground [meters]
- **zmax**: Distance from top of the block to ground [meters]
- **color**: Block color (see :ref:`color formatting <world_value_parsing>`)

Visual properties
^^^^^^^^^^^^^^^^^^^^^

All parameters from the :ref:`VisualObject <world_visual_object>` interface apply, including:

- **visual_scale**: Per-instance override for 3D model scale. If specified, this overrides 
  the ``model_scale`` defined in the visual model. Useful for creating size variations of 
  the same block class.

Example:

.. code-block:: xml

    <block class="tree" name="small_tree">
        <init_pose>10 20 0</init_pose>
        <visual_scale>0.5</visual_scale>  <!-- This tree is half size -->
    </block>
    
    <block class="tree" name="large_tree">
        <init_pose>15 25 0</init_pose>
        <visual_scale>2.0</visual_scale>  <!-- This tree is double size -->
    </block>


Block shape definition
=======================

Blocks can define their 2D collision shape in three ways:

1. Explicit polygon shape
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Define the block's footprint as a polygon using ``<shape>`` tag:

.. code-block:: xml

    <shape>
        <pt>-1.0 -0.5</pt>
        <pt>-1.0  0.5</pt>
        <pt> 1.0  0.5</pt>
        <pt> 1.0 -0.5</pt>
    </shape>

2. Automatic shape from visual model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``<shape_from_visual/>`` to automatically derive the collision shape from the 3D model's 
bounding box:

.. code-block:: xml

    <block class="building">
        <visual>
            <model_uri>building.dae</model_uri>
        </visual>
        <shape_from_visual/>
    </block>

.. note::
   When using ``<shape_from_visual/>``, the ``zmin`` and ``zmax`` values are also 
   automatically extracted from the 3D model.

3. Geometric primitives
^^^^^^^^^^^^^^^^^^^^^^^^^

Use simple geometric shapes with the ``<geometry>`` tag:

.. code-block:: xml

    <!-- Cylinder -->
    <geometry type="cylinder" radius="0.5" length="2.0" vertex_count="16"/>
    
    <!-- Sphere -->
    <geometry type="sphere" radius="0.5" vertex_count="16"/>
    
    <!-- Box -->
    <geometry type="box" lx="2.0" ly="1.0" lz="1.5"/>
    
    <!-- Ramp -->
    <geometry type="ramp" lx="2.0" ly="1.0" lz="0.5"/>
    
    <!-- Semi-cylinder bump -->
    <geometry type="semi_cylinder_bump" lx="1.0" ly="0.5" lz="0.2" vertex_count="10"/>

Supported geometry types:

- **cylinder**: Requires ``radius``, ``length``, optional ``vertex_count`` (default: 10)
- **sphere**: Requires ``radius``, optional ``vertex_count`` (default: 10)
- **box**: Requires ``lx`` (length), ``ly`` (width), ``lz`` (height)
- **ramp**: Requires ``lx``, ``ly``, ``lz``
- **semi_cylinder_bump**: Requires ``lx``, ``ly``, ``lz``, optional ``vertex_count``


Block instances
=======================

Each block class can be instantiated multiple times with the ``<block>`` tag:

.. code-block:: xml

    <block name="box1" class="wooden_box">
        <init_pose>5.0 3.0 0</init_pose>
    </block>
    
    <block name="box2" class="wooden_box">
        <init_pose>8.0 3.0 45</init_pose>
        <mass>75.0</mass>  <!-- Override class default -->
    </block>

Parameters in block instances
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **name**: Unique identifier for the block (required if not auto-generated)
- **class**: Reference to a ``<block:class>`` definition
- **init_pose**: Initial position in global coordinates: ``x y yaw_deg``
- Any parameter from the class definition can be overridden in the instance


Blocks without classes
========================

Blocks can also be defined inline without a class:

.. code-block:: xml

    <block name="standalone_obstacle">
        <init_pose>12.0 8.0 0</init_pose>
        <mass>100.0</mass>
        <static>true</static>
        <zmin>0.0</zmin>
        <zmax>2.5</zmax>
        
        <shape>
            <pt>-2.0 -1.0</pt>
            <pt>-2.0  1.0</pt>
            <pt> 2.0  1.0</pt>
            <pt> 2.0 -1.0</pt>
        </shape>
        
        <geometry type="box" lx="4.0" ly="2.0" lz="2.5"/>
    </block>


Complete example
=======================

.. code-block:: xml

    <?xml version="1.0"?>
    <world>
        <!-- Define a reusable block class -->
        <block:class name="pallet">
            <mass>25.0</mass>
            <zmin>0.0</zmin>
            <zmax>0.15</zmax>
            <color>#A0522D</color>
            <ground_friction>0.6</ground_friction>
            
            <visual>
                <model_uri>https://mrpt.github.io/mvsim-models/pallet.dae</model_uri>
                <model_scale>0.01</model_scale>
            </visual>
            
            <shape_from_visual/>
        </block:class>
        
        <!-- Create several instances with different poses and scales -->
        <for var="i" from="0" to="4">
            <block class="pallet" name="pallet_${i}">
                <init_pose>$f{i*2.0} 5.0 0</init_pose>
                <visual_scale>$f{0.8 + i*0.1}</visual_scale>
            </block>
        </for>
        
        <!-- Static barrier -->
        <block name="barrier" static="true">
            <init_pose>0 0 0</init_pose>
            <zmin>0.0</zmin>
            <zmax>1.2</zmax>
            <color>#FF0000</color>
            <geometry type="box" lx="10.0" ly="0.2" lz="1.2"/>
        </block>
    </world>


Advanced features
=======================

Dynamic blocks
^^^^^^^^^^^^^^^^^

Blocks with ``static="false"`` (the default) participate in the physics simulation 
and can be pushed, moved, or affected by forces:

.. code-block:: xml

    <block name="movable_crate" static="false">
        <init_pose>3.0 2.0 0</init_pose>
        <mass>50.0</mass>
        <ground_friction>0.7</ground_friction>
        <geometry type="box" lx="1.0" ly="1.0" lz="1.0"/>
    </block>

Intangible blocks
^^^^^^^^^^^^^^^^^^^

Blocks with ``intangible="true"`` are rendered visually but don't collide:

.. code-block:: xml

    <block name="hologram" intangible="true">
        <init_pose>0 0 0</init_pose>
        <visual>
            <model_uri>ghost.dae</model_uri>
        </visual>
    </block>

This is useful for decorative elements, visual markers, or semi-transparent objects.


Tips and best practices
=========================

1. **Use classes for reusability**: Define common obstacles as classes and instantiate them multiple times
2. **Leverage** ``visual_scale`` for variety: Create size variations without defining new classes
3. **Use** ``shape_from_visual`` when possible: Automatically derive collision shapes from 3D models
4. **Combine with loops**: Use ``<for>`` loops (see :ref:`flow control <world-flow-control>`) to create patterns of blocks
5. **Minimize collision complexity**: Use simple shapes when possible for better performance
6. **Mark decorative objects** as ``intangible`` to avoid unnecessary collision checks