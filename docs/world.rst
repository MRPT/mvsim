.. _world-definition-docs:

Defining the simulation world
=================================

Simulation happens inside an ``mvsim::World`` object. This is the central class for
usage from user code, running the simulation, loading XML models,
managing GUI visualization, etc. The ROS node acts as a bridge between
this class and the ROS subsystem, and the standalone ``mvsim`` cli tool
is just a thin wrapper loading a world model and running it.

Simulated worlds are described via configuration XML files
called **"world" files**.

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
   world_remote-resources
   world_flow-control
   world_value_parsing


1. "World elements"
---------------------

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

4. Vehicle class descriptions
--------------------------------

Tag **<vehicle:class>** depictd description of vehicle class. The
attribute *name* will be later referenced when describing vehicle
exemplars.

Inside **<vehicle:class>** tag, there are tags **<dynamics>**,
**<friction>** and exemplars of **<sensor>**.

Vehicle dynamics
^^^^^^^^^^^^^^^^

At the moment, there are three types of vehicle dynamics implemented.
Refer [vehicle\_models] for more information.

**<dynamics>** with attribute *class* specifies class of dynamics used.
Currently available classes:

-  differential

-  car\_ackermann

-  ackermann\_drivetrain

Each class has specific inner tags structure for its own configuration.

Common
^^^^^^

Every dynamics has wheels specified with tags **<i\_wheel>** where i
stand for wheel position index (r, l for differential drive and fr, fl,
rl, rr for Ackermann-drive)

Wheel tags have following attributes:

-  *pos* - two floats representing x an y coordinate of the wheel in
   local frame

-  *mass* - float value for mass of the wheel

-  *width* - float value representing wheel width [fig:wheel\_forces]

-  *diameter* - float value to represent wheel diameter
   [fig:wheel\_forces]

Ackermann models also use **<max\_steer\_ang\_deg>** to specify maximum
steering angle.

**<chassis>** is also common for all dynamics, it has attributes:

-  *mass* - mass of chassis

-  *zmin* - distance from bottom of the robot to ground

-  *zmax* - distance from top of the robot to ground

Controllers
^^^^^^^^^^^

There are controllers for every dynamics type [sec:controllers]. In XML
their names are

-  raw - control raw forces

-  twist\_pid - control with twist messages

-  front\_steer\_pid - [Ackermann only] - control with PID for velocity
   and raw steering angles

Controllers with *pid* in their names use PID regulator which needs to
be configured. There are tags **<KP><KI><KD>** for this purpose. Also
they need the parameter **<max\_torque>** to be set.

Twist controllers need to set initial **<V>** and **<W>** for linear and
angular velocities respectively.

Steer controllers need to set initial **<V>** and **<STEER\_ANG>** for
linear velocity and steering angle respectively.

Ackermann-drivetrain model
^^^^^^^^^^^^^^^^^^^^^^^^^^

needs a differential type and split to be configured. For this purpose
there is a tag **<drivetrain>** with argument *type*. Supported types
are defined in [sec:ackermann\_drivetrain]. In XML their names are:

-  open\_front

-  open\_rear

-  open\_4wd

-  torsen\_front

-  torsen\_rear

-  torsen\_4wd

**<drivetrain>** has inner tags describing its internal structure:

-  **<front\_rear\_split>**

-  **<front\_rear\_bias>**

-  **<front\_left\_right\_split>**

-  **<front\_left\_right\_bias>**

-  **<rear\_left\_right\_split>**

-  **<rear\_left\_right\_bias>**

which are pretty self-explanatory.

Friction
^^^^^^^^

Friction models are described in [sec:friction\_models] and defined
outside of **<dynamics>**. The tag for friction is **<friction>** with
attribute *class*.

Class names in XML are:

-  wardiagnemma

-  default

**Default** friction [sec:default\_friction] uses subtags:

-  **<mu>** - the friction coefficient

-  **<C\_damping>** - damping coefficient

In addition to **default**, **Ward-Iagnemma** friction includes subtags:

-  **A\_roll**

-  **R1**

-  **R2**

that are described in [sec:wi\_friction].

Sensors
^^^^^^^

Sensors are defined with **<sensor>** tag. It has attributes *type* and
*name*.

At the moment, only laser scanner sensor is implemented, its type is
*laser*. Subtags are:

-  **<pose>** - an MRPT CPose3D string value

-  **<fov\_degrees>** - FOV of the laser scanner

-  **<sensor\_period>** - period in seconds when sensor sends updates

-  **<nrays>** - laser scanner rays per FOV

-  **<range\_std\_noise>** - standard deviation of noise in distance
   measurements

-  **<angle\_std\_noise\_deg>** - standatd deviation of noise in angles
   of rays

-  **<bodies\_visible>** - boolean flag to see other robots or not


5. Vehicle instances
-------------------------

For each vehicle **class**, an arbitrary number of vehicle **instances**
can be created in a given world.

Vehicle instances are defined with the **<vehicle>** tag that has attributes
*name* and *class*. *class* must match one of the classes defined
earlier with **<vehicle:class>** tag.

Subtags are:

-  **<init\_pose>** - in global coordinates: :math:`x`, :math:`y`,
   :math:`\gamma` (deg)

-  **<init\_vel>** - in local coordinates: :math:`v_x`,\ :math:`v_y`,
   :math:`\omega` (deg/s)


6. "Obstacle block" classes
-----------------------------

Write me!


7. "Obstacle block" instances
-------------------------------

Write me!


8. Vehicles and blocks parameters
-----------------------------------

Vehicles and obstacles blocks share common C++ ``mvsim::Simulable`` and
``mvsim::VisualObject`` interfaces that provide the common parameters below.

.. note::

   The following parameters can appear in either the {vehicle,block} class
   definitions or in a particular instantiation block, depending on whether you
   want parameters to be common to all instances or not, respectively.


Related to topic publication
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Under the ``<publish> </publish>`` tag group:

- **publish\_pose\_topic**: If provided, the pose of this object will be published as a topic with message type ``mvsim_msgs::Pose``.
- **publish\_pose\_period**: Period (in seconds) for the topic publication.

Example:

.. code-block:: xml

	<publish>
	  <publish_pose_topic>/r1/pose</publish_pose_topic>
	  <publish_pose_period>50e-3</publish_pose_period>
	</publish>

Related to visual aspect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Under the ``<visual> </visual>`` tag group:

- **model\_uri**: Path to 3D model file. Can be any file format supported by ASSIMP,
  like ``.dae``, ``.stl``, etc. If empty, the default visual aspect will be used.
- **model\_scale**: (Default=1.0) Scale to apply to the 3D model.
- **model\_offset_x**, **model\_offset_y** , **model\_offset_z**: (Default=0) Offset translation [meters].
- **model\_yaw**, **model\_pitch**, **model\_roll**: (Default=0) Optional model rotation [degrees].
- **show_bounding_box**: (Default=``false``) Initial visibility of the object bounding box.

Example:

.. code-block:: xml

	<visual>
	  <model_uri>robot.obj</model_uri>
	  <model_scale>1.0</model_scale>
	  <model_offset_x>0.0</model_offset_x>
	  <model_offset_y>0.0</model_offset_y>
	  <model_offset_z>0.0</model_offset_z>
	</visual>


Simulation execution
========================

Simulation executes step-by-step with user-defined :math:`\Delta t` time
between steps. Each step has several sub steps:

-  Before time step - sets actions, updates models, etc.

-  Actual time step - updates dynamics

-  After time step - everything needed to be done with updated state


Logging
---------

Each vehicle is equipped with parameters logger(s). This logger is not
configurable and can be rewritten programmaticaly.

Logger are implemented via **CsvLogger** class and make log files in CSV
format which then can be opened via any editor or viewer.

Loggers control is introduced via robot controllers, each controller
controls only loggers of its robot.

Best results in visualizing offers QtiPlot [fig:qtiplot\_example1].

At the moment, following characteristics are logged:

-  Pose (:math:`x, y, z, \alpha, \beta, \gamma`)

-  Body velocity (:math:`\dot{x}, \dot{z}, \dot{z}`)

-  Wheel torque (:math:`\tau`)

-  Wheel weight (:math:`m_{wp}`)

-  Wheel velocity (:math:`v_x, v_y`)

Loggers support runtime clear and creating new session. The new session
mode finalizes current log files and starts to write to a new bunch of
them.


Limitations
-------------

-  A limitation of box2d is that no element can be thinner than 0.05 units, or
   the following assert will be raised while loading the world model:

.. code-block::

	Box2D/Box2D/Collision/Shapes/b2PolygonShape.cpp:158: void b2PolygonShape::Set(const b2Vec2*, int32): Assertion `false' failed.
