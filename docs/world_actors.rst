.. _world_actors:

================
Animated Actors
================

Animated actors are skeletal-animated 3D characters (pedestrians, workers, etc.) that
follow predefined waypoint paths in the simulation world. They are purely kinematic:
they do not respond to external forces or interact with the physics engine, but they
provide realistic visual motion through bone-driven animation (idle, walk, run). 

They are included into collision detection and can be sensed by all visual, range or LiDAR sensors, though.

Actors use the same class/instance pattern as :ref:`blocks <world_blocks>`: you first
define reusable **actor classes** (``<actor:class>``), then instantiate them with
per-instance overrides (``<actor>``).

.. contents::
   :depth: 1
   :local:
   :backlinks: none


Animated 3D Models
~~~~~~~~~~~~~~~~~~

Actor animation relies on skeletal data embedded in the 3D model file (FBX, GLB, glTF,
Collada, etc.). The model must contain at least one animation clip with bone keyframes.

A free set of animated human characters ready to use:

.. code-block:: xml

   <model_uri>https://mrpt.github.io/mvsim-models/AnimatedMenCharacters.zip/FBX/Male_Casual.fbx</model_uri>
   <model_uri>https://mrpt.github.io/mvsim-models/AnimatedWomenCharacters.zip/FBX/Female_Casual.fbx</model_uri>

Available models (you can download the ZIP files above and visualize locally):


.. code-block:: bash

    Female_Alternative.fbx
    Female_Casual.fbx
    Female_Dress.fbx
    Female_TankTop.fbx
    Smooth_Female_Alternative.fbx
    Smooth_Female_Casual.fbx
    Smooth_Female_Dress.fbx
    Smooth_Female_TankTop.fbx
    Male_Casual.fbx
    Male_LongSleeve.fbx
    Male_Shirt.fbx
    Male_Suit.fbx
    Smooth_Male_Casual.fbx
    Smooth_Male_LongSleeve.fbx
    Smooth_Male_Shirt.fbx
    Smooth_Male_Suit.fbx


The available animation clip names for this model pack are:

* ``HumanArmature|Man_Clapping``
* ``HumanArmature|Man_Death``
* ``HumanArmature|Man_Idle``
* ``HumanArmature|Man_Jump``
* ``HumanArmature|Man_Punch``
* ``HumanArmature|Man_Run``
* ``HumanArmature|Man_RunningJump``
* ``HumanArmature|Man_Sitting``
* ``HumanArmature|Man_Standing``
* ``HumanArmature|Man_SwordSlash``
* ``HumanArmature|Man_Walk``

.. note::
   Animation clip names are model-specific. MVSim will log all discovered clip names
   at load time to help you find the correct strings.

-------

|

Actor Class Definition
~~~~~~~~~~~~~~~~~~~~~~

**<actor:class name="...">** defines a reusable actor template with default appearance,
speed, and animation mapping. Multiple ``<actor>`` instances can share one class.

**XML Configuration:**

.. code-block:: xml

   <actor:class name="pedestrian">
       <walking_speed>1.4</walking_speed>
       <running_speed>3.5</running_speed>
       <height>1.75</height>
       <collision_radius>0.3</collision_radius>
       <animation_idle>HumanArmature|Man_Idle</animation_idle>
       <animation_walk>HumanArmature|Man_Walk</animation_walk>
       <animation_run>HumanArmature|Man_Run</animation_run>
       <visual>
           <model_uri>https://mrpt.github.io/mvsim-models/AnimatedMenCharacters.zip/FBX/Male_Casual.fbx</model_uri>
           <model_scale>0.0027</model_scale>
           <model_roll>90.0</model_roll>
       </visual>
   </actor:class>

**Subtags:**

* **<walking_speed>** - Walking speed in m/s (default: 1.4)
* **<running_speed>** - Running speed in m/s (default: 3.5)
* **<turning_rate>** - Turning rate in deg/s (default: 120)
* **<height>** - Character height in meters (default: 1.75). Used for the placeholder visual
* **<collision_radius>** - Radius for collision shape in meters (default: 0.3)
* **<collision_height>** - Height for collision shape in meters (default: 1.7)
* **<animation_idle>** - Name of the idle animation clip in the 3D model
* **<animation_walk>** - Name of the walking animation clip
* **<animation_run>** - Name of the running animation clip
* **<visual>** - 3D model specification (see :ref:`world_visual_object`). Supports the
  standard subtags ``<model_uri>``, ``<model_scale>``, ``<model_offset_x>``, etc.

-------

|

Actor Instance
~~~~~~~~~~~~~~

**<actor name="..." class="...">** creates one actor in the world. Per-instance subtags
override the class defaults. Each actor can define its own waypoint path.

**XML Configuration:**

.. code-block:: xml

   <actor name="person1" class="pedestrian">
       <init_pose>5 10 0</init_pose>
       <path loop="true">
           <waypoint>5 10 0</waypoint>
           <waypoint>15 10 0</waypoint>
           <waypoint pause="2.0">15 10 180</waypoint>
           <waypoint animation="run">5 10 180</waypoint>
       </path>
   </actor>

**Attributes:**

* **name** - Unique name for the actor instance
* **class** - Name of a previously defined ``<actor:class>``

**Subtags:**

* **<init_pose>** - Initial pose as ``x y yaw_deg`` (see :ref:`world_simulable`)
* Any ``<actor:class>`` subtag can be repeated here to override the class default
* **<path>** - Waypoint path definition (see below)

-------

|

Waypoint Paths
~~~~~~~~~~~~~~

The **<path>** subtag defines a sequence of waypoints that the actor will follow
automatically during the simulation.

**Path attributes:**

* **loop** - ``"true"`` or ``"false"``. When true, the actor returns to the first
  waypoint after reaching the last one (default: true)

**Waypoint format:**

Each **<waypoint>** contains a pose string ``x y yaw_deg`` and optional attributes:

* **pause** - Time in seconds to pause at this waypoint before continuing
* **animation** - Override the automatic animation selection for the segment leading
  to this waypoint. Values: ``"walk"`` (default), ``"run"``, ``"idle"``

**Example — Looping patrol with a pause and a running segment:**

.. code-block:: xml

   <path loop="true">
       <waypoint>0 0 0</waypoint>
       <waypoint>20 0 0</waypoint>
       <waypoint pause="3.0">20 10 90</waypoint>
       <waypoint animation="run">0 10 180</waypoint>
       <waypoint>0 0 270</waypoint>
   </path>

The actor automatically selects the appropriate animation clip (idle, walk, or run)
based on its current movement speed. When paused at a waypoint, it plays the idle
animation. The ``animation`` attribute on a waypoint forces a specific clip for
the segment leading to that waypoint.

-------

|

Animation State Machine
~~~~~~~~~~~~~~~~~~~~~~~

Each actor has an internal state machine that selects the active animation clip:

* **Idle** — played when the actor is stationary or paused at a waypoint
* **Walking** — played when moving at or below the walking speed
* **Running** — played when moving above 80% of the running speed

Animation playback speed is automatically scaled to match actual movement velocity,
so the walk cycle stays synchronized with the character's ground speed.

-------

|

Placeholder Visual
~~~~~~~~~~~~~~~~~~

If no ``<visual>`` is defined (or the model file cannot be loaded), the actor is
rendered as a simple cylinder+sphere placeholder whose dimensions are derived from
``<collision_radius>`` and ``<collision_height>``.

-------

|

Complete Example
~~~~~~~~~~~~~~~~

.. code-block:: xml

   <mvsim_world version="1.0">

       <actor:class name="pedestrian">
           <walking_speed>1.4</walking_speed>
           <animation_walk>HumanArmature|Man_Walk</animation_walk>
           <animation_idle>HumanArmature|Man_Idle</animation_idle>
           <animation_run>HumanArmature|Man_Run</animation_run>
           <visual>
               <model_uri>https://mrpt.github.io/mvsim-models/AnimatedMenCharacters.zip/FBX/Male_Casual.fbx</model_uri>
               <model_scale>0.0027</model_scale>
               <model_roll>90.0</model_roll>
           </visual>
       </actor:class>

       <actor name="walker1" class="pedestrian">
           <init_pose>0 0 0</init_pose>
           <path loop="true">
               <waypoint>0 0 0</waypoint>
               <waypoint>10 0 0</waypoint>
               <waypoint pause="2.0">10 5 90</waypoint>
               <waypoint>0 5 180</waypoint>
           </path>
       </actor>

       <actor name="runner1" class="pedestrian">
           <init_pose>-5 -5 45</init_pose>
           <path loop="true">
               <waypoint>-5 -5 45</waypoint>
               <waypoint animation="run">5 5 45</waypoint>
               <waypoint pause="1.0" animation="idle">5 5 225</waypoint>
               <waypoint animation="run">-5 -5 225</waypoint>
           </path>
       </actor>

   </mvsim_world>
