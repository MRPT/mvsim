.. _first-steps:

First steps
===================

After installing or building from sources, you are ready to test the
simulator to get used to MVSIM.

Launch it
------------

We will launch a demo with a Jackal mobile robot in a warehouse.
The commands required to launch it depends on the installation method:

.. tab-set::

    .. tab-item:: ROS 1

        If you installed MVSim as a ROS 1 package, just run:

        .. code-block:: bash

            roslaunch mvsim ros1_warehouse.launch

    .. tab-item:: ROS 2
        :selected:

        If you installed MVSim as a ROS 2 package, just run:

        .. code-block:: bash

            ros2 launch mvsim demo_warehouse.launch.py

    .. tab-item:: Standalone MVSim build

        Assuming you compiled MVSim in the directory ``MVSIM_ROOT``,
        with cmake build directory ``build-Release``, run:

        .. code-block:: bash

            cd MVSIM_ROOT
            build-Release/bin/mvsim launch mvsim_tutorial/demo_warehouse.world.xml


You should see the GUI of a demo world with a robot equipped with a 3D lidar and,
if using the ROS version, an RViz window with a visualization of some of the the
published sensor topics.

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mvsim-warehouse-teleop-demo.mp4" type="video/mp4">
     </video>
   </div>


Move it
------------

If you are running MVSim as a ROS node, you can launch any standard teleoperation node
and send motion commands to ``/cmd_vel`` as with any other robot or simulator.

If you want to use the teleop panel in rviz2, please install `visualization_tutorials <https://github.com/ros-visualization/visualization_tutorials/tree/ros2>`_.

.. code-block:: bash

    cd ros2_ws/src
    git clone -b ros2 https://github.com/ros-visualization/visualization_tutorials
    colcon build --symlink-install

Additionally, MVSim allows you to **move the robot directly using the keyboard or a joystick**.
Make sure of giving the focus to the MVSim window first,
then use these keys:

- ``w/s`` to increase/decrease the PI controller setpoint linear speed, and
- ``a/d`` to change the corresponding angular speed, that is, rotate to the left and right.
- Use the spacebar as a brake.
- In worlds with more than one robot, select the active robot by pressing the numeric
  keys ``1``, ``2``, etc.

All the details on **keyboard and joystick-based control** are listed `here <teleoperation.html>`_.

Next, you can jump straight into the many other demo worlds and launch files
available under `mvsim_tutorial <https://github.com/MRPT/mvsim/tree/master/mvsim_tutorial>`_,
or continue reading these docs to better understand them.

