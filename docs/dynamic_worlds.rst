.. _dynamic-worlds:

Building worlds at runtime
=============================

Robots, obstacles and other world elements can be **inserted into a running
simulation, and removed later**, without reloading the world. For example, to
place the obstacles of each trial of an experiment, to add robots as a test
requires them, or to build a whole scenario from an external program.

Entities are given as **world XML**: the same tags as in a ``*.world.xml`` file.
They are complete simulation entities: they collide, their sensors (including
cameras and LiDARs) produce data, and they show up in the GUI. When removed,
their sensors, physics bodies, joints and visualization go away too.

For lightweight visual-only objects, such as marks drawn on the ground or
debug overlays, see :ref:`runtime-objects` instead.

|

What can be inserted
----------------------

Any tags valid in a world file, optionally enclosed in a ``<mvsim_world>`` root
tag:

- Entities: ``<vehicle>``, ``<block>`` and ``<element>`` (e.g. a
  ``horizontal_plane`` or an ``occupancy_grid``).
- Helpers: ``<include>``, ``<vehicle:class>``, ``<block:class>``,
  ``<variable>``, ``<for>`` and ``<if>``.

Rules:

- Relative paths (e.g. in ``<include>``) are resolved with respect to the
  directory of the world file loaded by the simulator, unless another base
  directory is given.
- Names must be unique. Elements have no name in world files: when inserted at
  runtime, they get a unique ``element_<N>`` name, so they can be removed.
- Insertion is all or nothing: if anything fails (e.g. an XML error or a name
  already in use), nothing is inserted.
- Removal works by name, for vehicles, blocks, elements, and also
  :ref:`runtime objects <runtime-objects>`.

A world with just the ground, to build everything at runtime, is provided as
``mvsim_tutorial/demo_empty.world.xml``.

|

From ROS 2
------------

The ``mvsim_node`` offers the standard simulator services of the
`simulation_interfaces <https://github.com/ros-simulation/simulation_interfaces>`_
package (if it was found when building MVSim):

=============================  =====================================================================
Service                        Description
=============================  =====================================================================
``spawn_entity``               Inserts entities from MVSim world XML (``resource_string``), or from
                               a file path or ``file://`` URI.
``delete_entity``              Removes an entity by name.
``get_entities``               Names of all entities, optionally filtered by a regular expression.
``get_entity_state``           Pose and velocity of an entity, in the ``world_frame_id`` frame.
``set_entity_state``           Moves an entity, and/or sets its velocity.
``get_simulator_features``     Lists the supported features.
=============================  =====================================================================

The ``spawn_entity`` request fields work as follows:

- ``name`` (optional) and ``initial_pose`` (if it is not the identity) override
  those in the XML, which must then define exactly one ``<vehicle>``,
  ``<block>`` or ``<element>``. With ``initial_pose``, ``<init_pose>`` is not
  required in the XML.
- ``allow_renaming``: if the name is taken, a suffix ``_1``, ``_2``... is added
  instead of failing.
- ``entity_namespace`` must be empty or equal to the name: vehicles inserted at
  runtime always publish their topics under their name (e.g. ``/bot1/cmd_vel``).
- The response ``entity_name`` lists the names of all inserted entities,
  separated by commas.

Example: start an empty world, then insert a robot with a 2D LiDAR and a
camera, drive it, and remove it:

.. code-block:: bash

   ros2 launch mvsim launch_world.launch.py \
     world_file:=$(ros2 pkg prefix mvsim)/share/mvsim/mvsim_tutorial/demo_empty.world.xml

   # In another terminal:
   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity \
     "{name: bot1, entity_resource: {resource_string: '
         <include file=\"../definitions/jackal.vehicle.xml\" default_sensors=\"true\"/>
         <vehicle name=\"bot1\" class=\"jackal\"/>'},
       initial_pose: {pose: {position: {x: 2.0, y: 1.0}}}}"

   ros2 topic pub -r 10 /bot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

   ros2 service call /get_entity_state simulation_interfaces/srv/GetEntityState \
     "{entity: bot1}"

   ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity \
     "{entity: bot1}"

Several entities in one call, e.g. a row of boxes made with a ``<for>`` loop:

.. code-block:: bash

   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity \
     "{entity_resource: {resource_string: '
       <for var=\"i\" from=\"0\" to=\"4\">
         <block name=\"box_\${i}\">
           <static>true</static> <zmin>0</zmin> <zmax>0.5</zmax> <color>#a06020</color>
           <shape><pt>-0.25 -0.25</pt><pt>0.25 -0.25</pt><pt>0.25 0.25</pt><pt>-0.25 0.25</pt></shape>
           <init_pose>\$f{2+\${i}} -2 0</init_pose>
         </block>
       </for>'}}"

   ros2 service call /get_entities simulation_interfaces/srv/GetEntities \
     "{filters: {filter: '^box_'}}"

From a file (``uri`` with a file path or a ``file://`` URI; relative paths in it
are resolved wrt the file directory):

.. code-block:: bash

   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity \
     "{entity_resource: {uri: '/path/to/my_obstacles.xml'}}"

Move an object:

.. code-block:: bash

   ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState \
     "{entity: box_0, set_pose: true, state: {pose: {position: {x: -5.0, y: -5.0}}}}"

.. note::

   - With ``simulation_interfaces`` older than 2.0 (ROS 2 Humble and Jazzy), the
     spawn resource fields are named ``uri`` and ``resource_string`` (no
     ``entity_resource``), and ``set_entity_state`` has no ``set_pose`` /
     ``set_twist`` flags: it always sets both.
   - Service names are relative to the node namespace, so they can be remapped.
   - Robots loaded with the world keep their usual topic names: without a
     namespace if the world has only one vehicle, unless
     ``force_publish_vehicle_namespace`` is set.

|

From Python or any ZMQ client
-------------------------------

Services ``insert_entities`` (``SrvInsertEntities``: the XML, and optionally a
base directory for relative paths; it returns the names of the new entities)
and ``remove_entities`` (``SrvRemoveEntities``, by names).

.. code-block:: python

    from mvsim_comms import pymvsim_comms
    from mvsim_msgs import SrvInsertEntities_pb2, SrvInsertEntitiesAnswer_pb2
    from mvsim_msgs import SrvRemoveEntities_pb2, SrvRemoveEntitiesAnswer_pb2

    client = pymvsim_comms.mvsim.Client()
    client.setName("my-app")
    client.connect()

    req = SrvInsertEntities_pb2.SrvInsertEntities()
    req.xml = """
      <include file="../definitions/jackal.vehicle.xml" default_sensors="false" />
      <vehicle name="bot1" class="jackal"> <init_pose>2 2 0</init_pose> </vehicle>
    """
    ans = SrvInsertEntitiesAnswer_pb2.SrvInsertEntitiesAnswer()
    ans.ParseFromString(client.callService('insert_entities', req.SerializeToString()))
    print(ans.success, ans.errorMessage, list(ans.names))

    req = SrvRemoveEntities_pb2.SrvRemoveEntities()
    req.names.append('bot1')
    ans = SrvRemoveEntitiesAnswer_pb2.SrvRemoveEntitiesAnswer()
    ans.ParseFromString(client.callService('remove_entities', req.SerializeToString()))

A complete example, which also drives the new robot, is
``examples_python/insert-entities-example.py``:

.. code-block:: bash

   mvsim launch mvsim_tutorial/demo_empty.world.xml &
   python3 examples_python/insert-entities-example.py

|

From C++
-----------

.. code-block:: cpp

    #include <mvsim/World.h>

    // Returns the names of the new entities; throws on errors:
    const auto names = world.insertEntitiesFromXML(R"(
        <block name="box1">
          <static>true</static> <zmin>0</zmin> <zmax>1</zmax>
          <shape><pt>-0.5 -0.5</pt><pt>0.5 -0.5</pt><pt>0.5 0.5</pt><pt>-0.5 0.5</pt></shape>
          <init_pose>2 0 0</init_pose>
        </block>)");

    // Overriding the name and pose of a single entity:
    mvsim::World::InsertOptions opts;
    opts.name = "box2";
    opts.pose = mrpt::math::TPose3D(4.0, 1.0, 0, 0, 0, 0);
    world.insertEntitiesFromXML(xmlOfOneBlock, opts);

    world.removeEntity("box1");  // false if not found

    // To react to insertions and removals (e.g. to create ROS publishers):
    world.registerCallbackOnEntityChange(
        [](const mvsim::World::EntityChange& c)
        { std::cout << (c.added ? "added: " : "removed: ") << c.name << "\n"; });

``insertEntitiesFromXML()`` and ``removeEntity()`` must not run at the same time
as ``run_simulation()``: call them from the thread that runs the simulation, or
from any other thread through ``runInSimulationThread()``, which runs them at the
start of the next ``run_simulation()`` call:

.. code-block:: cpp

    auto fut = world.runInSimulationThread([&]() { world.removeEntity("box2"); });
    fut.get();  // waits for it, and rethrows its exceptions
