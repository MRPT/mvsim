.. .. _installing:

Installing
===========

From ROS 1 or ROS 2 repositories
--------------------------------

**Recommended**: This is probably the easiest way to install MVSim and all the example files.

In Debian/Ubuntu systems, activate your ROS environment (``setup.bash``) if not done automatically 
in your ``~./bashrc`` file, then just run:

.. code-block:: bash

    sudo apt install ros-$ROS_DISTRO-mvsim

Check the `build status table <https://github.com/MRPT/mvsim#build-matrix-status>`_ to find out
what MVSim version is available for your ROS distribution.

Now, you can see `first steps <first-steps.html>`_ on how to launch demo files.

.. note::
    You can also build MVSim **ROS nodes** from sources, by cloning into a catkin or colcon workspace
    and build as usual (`catkin build` or `colcon build`).


Build from sources
----------------------

Clone the git repository, including the submodules:

.. code-block:: bash

 git clone https://github.com/MRPT/mvsim.git --recursive


You will need these dependencies:

- A decent C++17 compiler.
- CMake >= 3.9
- MRPT (>=2.5.6): In Windows, `build from sources <https://docs.mrpt.org/reference/latest/compiling.html>`_
  or `get the precompiled installer <https://docs.mrpt.org/reference/latest/download-mrpt.html>`_.
- Box2D (>=2.4): It will use an embedded copy (git submodule) if no (or too old) system version is found.

In Ubuntu, all requirements can be satisfied executing:

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


Compile as usual in CMake projects:

.. code-block:: bash

 mkdir build
 cd build
 cmake ..
 make
 #make test

.. note::
   The instructions above will build the MVSim standalone CLI application, the C++ and the Python libraries.
   If you also want the **ROS nodes**, make sure of having your ROS system activated (having sourced ``setup.bash``)
   at the time of invoking the cmake configuration (``cmake ..``). In that case, using ``colcon`` or ``catkin`` as usual
   is strongly recommended instead of manually invoking cmake. Just put MVSim under the workspace ``src`` directory as 
   any other ROS package.
