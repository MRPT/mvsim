.. _runtime-objects:

Runtime objects and ground truth
=====================================

Visual-only objects can be spawned, moved and removed while the simulation
runs, without reloading the world. Typical uses: marks drawn on the ground by a
robot tool, targets placed for each trial, or debug overlays.

These objects have **no physics** (no collisions, like ``intangible`` blocks).
By default they are rendered both in the GUI and in **camera and RGB-D sensor
images**; GUI-only overlays are also possible.

|

Shapes
---------

All shapes are defined in a local frame, placed at the object ``pose`` in world
coordinates:

======================  =================================================================
Shape                   Geometry
======================  =================================================================
``Box``                 ``size=(lx,ly,lz)``, centered at the pose.
``Cylinder``            ``size=(diameter,-,height)``, its base at the pose.
``Sphere``              ``size=(diameter,-,-)``, centered at the pose.
``Rectangle``           Flat decal in the local XY plane, ``size=(lx,ly,-)``. Optionally textured with an image file.
``Disk``                Flat decal, ``size=(diameter,-,-)``.
``Polygon``             Flat decal: a convex polygon in the local XY plane.
``Triangles``           Arbitrary mesh, 3 points per triangle, optional per-point colors.
``Lines``               Line segments, 2 points per segment. ``size.x`` is the width in pixels.
======================  =================================================================

To avoid z-fighting with the ground, give flat decals a small height (e.g. 2-5 mm).
With ``on_ground=true``, ``pose.z`` is understood as the height over the ground
surface under the object, which is useful on uneven terrain.

Spawning an object with an existing name replaces it. Spawning and removing many
objects is cheap, and many objects can be spawned in one call.

|

From C++
-----------

.. code-block:: cpp

    #include <mvsim/World.h>

    mvsim::RuntimeObjectDescription d;
    d.name = "marks/1";
    d.shape = mvsim::RuntimeObjectDescription::Shape::Rectangle;
    d.pose = {1.0, 2.0, 0.003, 0, 0, 0};
    d.size = {0.5, 0.2, 0};
    d.color = {0xff, 0xff, 0xff};
    world.runtimeObjects().spawn(d);  // or a std::vector<> of them

    world.runtimeObjects().setPose("marks/1", {1.0, 2.5, 0.003, 0, 0, 0});
    world.runtimeObjects().removeByPrefix("marks/");

All these methods are thread-safe. Changes are reflected in the next rendered
sensor image.

|

From Python or any ZMQ client
-------------------------------

Services ``spawn_objects`` (``SrvSpawnObjects``, a list of ``RuntimeObject``),
``remove_objects`` (``SrvRemoveObjects``, by names and/or a name prefix), and
also ``set_pose`` / ``get_pose``, which work for runtime objects too.
See ``examples_python/spawn-objects-example.py``.

|

From ROS 2
------------

The ``mvsim_node`` subscribes to two ``visualization_msgs/MarkerArray`` topics:

- ``runtime_objects``: objects seen by sensors.
- ``runtime_overlays``: GUI-only objects.

Markers must be given in the world frame (``world_frame_id`` parameter, default
``map``). Their ``ns`` and ``id`` identify each object, and ``ADD``/``MODIFY``,
``DELETE`` and ``DELETEALL`` actions are supported (``DELETEALL`` removes all
the objects created from that topic). Supported marker types:

- ``CUBE``: a box, or a flat rectangle decal if ``scale.z`` is zero.
- ``CYLINDER``: a cylinder, or a flat disk decal if ``scale.z`` is zero.
- ``SPHERE``, ``TRIANGLE_LIST``, ``LINE_LIST``, ``LINE_STRIP``.

The ``lifetime`` field is ignored: objects remain until deleted.

Example:

.. code-block:: bash

    ros2 topic pub --once /runtime_objects visualization_msgs/msg/MarkerArray \
      "{markers: [{header: {frame_id: map}, ns: marks, id: 1, type: 1, action: 0,
        pose: {position: {x: 1.0, y: 2.0, z: 0.003}, orientation: {w: 1.0}},
        scale: {x: 0.5, y: 0.5, z: 0.0}, color: {r: 1.0, a: 1.0}}]}"

|

Ground truth of all objects
-----------------------------

The poses of all named objects (vehicles, blocks, actors and runtime objects)
can be retrieved as a consistent snapshot at the end of a simulation step:

- C++: ``World::getGroundTruthSnapshot()``.
- ZMQ: service ``get_all_poses`` (``SrvGetAllPoses``), which also returns the
  simulation time and each object velocity.
- ROS 2: set the ``mvsim_node`` parameter ``objects_ground_truth_rate`` (Hz,
  default ``0``: disabled) to publish a ``tf2_msgs/TFMessage`` on the
  ``objects_ground_truth`` topic, one transform per object from
  ``world_frame_id`` to the object name, stamped with simulation time. It is
  published on its own topic (not ``/tf``) to avoid clashing with the robots' TF
  trees.
