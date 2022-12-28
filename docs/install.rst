Installing
===========

From ROS1 or ROS2 repositories
--------------------------------

This is preferred for users to quickly and easily install MVSIM and all the example files.

Once you have activated your ROS environment (`setup.bash`), just run:

.. code-block:: bash

    sudo apt install ros-$ROS_DISTRO-mvsim

See `next steps <first-steps.html>`_ on how to launch demo files.


.. note::
    You can also build MVSIM ROS nodes from sources, by cloning into a catkin or colcon workspace
    and build as usual (`catkin build` or `colcon build`).


Build from sources
----------------------

Dependencies:

- A decent C++17 compiler.
- CMake >= 3.9
- MRPT (>=2.5.6): In Windows, build from sources or install precompiled binaries.
- Box2D (>=2.4): It will use an embedded copy (git submodule) if no (or too old) system version is found.

In Ubuntu, this will install all requirements:

.. code-block:: bash

 sudo apt install \
   build-essential cmake g++ \
   libbox2d-dev \
   libmrpt-opengl-dev libmrpt-obs-dev libmrpt-maps-dev libmrpt-tclap-dev \
   libmrpt-gui-dev libmrpt-tfest-dev \
   protobuf-compiler \
   libzmq3-dev \
   pybind11-dev \
   libprotobuf-dev \
   libpython3-dev 


Compile as usual:

.. code-block:: bash

 mkdir build
 cd build
 cmake ..
 make
 #make test


