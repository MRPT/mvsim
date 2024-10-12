Demo worlds
===================

This page describes the worlds provided as examples ready to run within
MVSim.

.. note::
   The 3D models used in these demos belong to either the `MVSim main repository <https://github.com/MRPT/mvsim>`_
   or the `mvsim-models <https://github.com/MRPT/mvsim-models>`_ repository (and downloaded automatically 
   via :ref:`world_remote-resources`). In any case, each model is accompanied by
   its corresponding LICENSE and attribution to the author(s).

.. contents:: List of demos
   :depth: 1
   :local:
   :backlinks: none


demo_warehouse
---------------------

Example of a 3D Lidar Jackal robot in a warehouse. The XML illustrates how to animate an object to make it to follow a given trajectory
in a loop.

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_warehouse.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_warehouse.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_warehouse.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_warehouse.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_warehouse.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_warehouse.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_warehouse.world.xml
      :language: xml

demo_road_circuit1
---------------------

Example of two Jackal robots with cameras and 3D LiDARs in a 2.5 world, including ramps, road bumps, and a bridge.
For ROS, note that having multiple robots uses namespaces even for ``/tf``, hence the ``remappings`` (see the launch file)
needed for RViz to show the sensors and send ``/cmd_vel`` commands.

.. tab-set::
    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_road_circuit1.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_road_circuit1.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim-demo-road-circuit1_world.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_road_circuit1.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_road_circuit1.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_road_circuit1.world.xml
      :language: xml


demo_1robot
------------------

A simple 2D world defined via an occupancy grid map and a robot equipped with 2D lidars.

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_1robot.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_1robot.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_1robot.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_1robot.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_1robot.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_1robot.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_1robot.world.xml
      :language: xml


demo_2robots
------------------

A world with 2 simple robots and a couple of custom "blocks" (furniture).

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_2robots.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_2robots.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_2robots.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_2robots.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_2robots.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_2robots.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_2robots.world.xml
      :language: xml



demo_camera
------------------

Example of a robot with a camera sensor (RGB).

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_camera.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_camera.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_camera.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_camera.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_camera.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_camera.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_camera.world.xml
      :language: xml



demo_depth_camera
------------------

Example of a robot with a depth camera sensor (RGB+D).

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_depth_camera.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_depth_camera.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_depth_camera.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_depth_camera.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_depth_camera.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_depth_camera.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_depth_camera.world.xml
      :language: xml


demo_elevation_map
---------------------

Example of a 3D Lidar robot in a "2.5D" world defined by an elevation map.

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_elevation_map.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_elevation_map.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_elevation_map.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_elevation_map.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_elevation_map.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_elevation_map.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_elevation_map.world.xml
      :language: xml


demo_greenhouse
---------------------

Example of a 3D Lidar robot in a greenhouse. The XML illustrates features
such as XML-level variables, XML-for loops for repetitive patterns of objects,
slightly randomized-perturbations in plant poses, etc.

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_greenhouse.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_greenhouse.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_greenhouse.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_greenhouse.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_greenhouse.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_greenhouse.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_greenhouse.world.xml
      :language: xml


demo_turtlebot_world
---------------------

The MVSim port of the classic ROS "turtlebot world" scenario. 
The XML illustrates how to define obstacle blocks with basic geometric shapes without
external ``.dae`` or ``.stl`` files.

.. tab-set::
    .. tab-item:: ROS 1

        .. code-block:: bash

            roslaunch mvsim demo_turtlebot_world.launch

    .. tab-item:: ROS 2
        :selected:

        .. code-block:: bash

            ros2 launch mvsim demo_turtlebot_world.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_turtlebot_world.world.xml


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim_demo_turtlebot_world.mp4" type="video/mp4">
     </video>
   </div>


.. dropdown:: World XML code

   File: `mvsim_tutorial/demo_turtlebot_world.world.xml <https://github.com/MRPT/mvsim/blob/develop/mvsim_tutorial/demo_turtlebot_world.world.xml>`_

   .. literalinclude:: ../mvsim_tutorial/demo_turtlebot_world.world.xml
      :language: xml


