.. _world-definition-docs:

Defining the simulation world
=================================

Simulation happens inside an ``mvsim::World`` object. This is the central class for
usage from user code, running the simulation, loading XML models,
managing GUI visualization, etc. The ROS node acts as a bridge between
this class and the ROS subsystem, and the standalone ``mvsim`` cli tool
is just a thin wrapper loading a world model and running it.

Simulated worlds are described via configuration XML files
called **"world" files**, which describes both,
the environment itself and the vehicles or robots that move in it.
By means of :ref:`includes <world-includes>`, robots, sensors, or environment objects
can be defined in their own files for the sake of reusability.

.. note:: 
   Many examples can be found in the `mvsim_tutorial directory <https://github.com/MRPT/mvsim/tree/master/mvsim_tutorial>`_.
   Look for the ``*.world.xml`` files.


.. figure:: https://mrpt.github.io/mvsim-models/anims/mvsim-warehouse.gif

   Demo "warehouse" world.


The next pages cover the **main different parts** of a world file, so you can understand the
provided examples, modify them, or **create your own worlds and robots**.


Global settings
-----------------------

First, we have global definitions on the simulation itself, the GUI, and the lights and shadows:

.. toctree::
   :maxdepth: 2

   world_global
   world_gui
   world_georeference
   world_lighting

World contents
-----------------------

Then we have to populate the world. MVSim defines **three kinds of objects**: 

1. **Elements** (like walls, the ground, etc.) which normally do not move, 
2. **Blocks**: most normally, obstacles, furniture, etc. Any object that may move or not, but which is not a controllable robot/vehicle; and 
3. **Vehicles**: the robots/vehicles/agents themselves.

Both, blocks and vehicles share two common APIs or interfaces: ``Simulable`` and ``VisualObject``, hence
the properties of such interfaces are explained below in independent pages:


.. toctree::
   :maxdepth: 2

   world_elements
   world_blocks
   world_vehicles
   world_simulable
   world_visual_object


Advanced features
-----------------------

Other key features of MVSim world files are summarized next:

.. toctree::
   :maxdepth: 2

   world_includes
   world_xml_parser
   world_remote-resources
   world_flow-control
   world_value_parsing


Simulation execution
----------------------

Simulation executes step-by-step with user-defined :math:`\Delta t` time
between steps. Each step comprises:

-  Before time step - sets actions, updates models, etc.

-  Actual time step - updates dynamics

-  After time step - everything needed to be done with updated state


Simulation limitations
------------------------

-  A limitation of box2d is that no element can be thinner than 0.05 units, or
   the following assert will be raised while loading the world model:

.. code-block::

	Box2D/Box2D/Collision/Shapes/b2PolygonShape.cpp:158: void b2PolygonShape::Set(const b2Vec2*, int32): Assertion `false' failed.

