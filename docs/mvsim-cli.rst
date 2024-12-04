mvsim cli
===================

The command-line interface (cli) application ``mvsim`` is the main
entry-point to run, query, or modify a running simulation from a 
command line terminal. It can be used to launch standalone simulations (without ROS), 
or to modify, manipulate, or query a running simulation within a ROS system.

.. note::
   The ROS node is a different cli application, named ``mvsim_node`` instead.
   The ROS package ``mvsim`` ships **both** executables: ``mvsim`` and ``mvsim_node``.
   See :ref:`mvsim ROS node`.

Invoking ``mvsim``
------------------------
Make sure you have compiled or installed MVSim, and that the executable named ``mvsim`` is 
in your ``PATH``. If you installed it as a ROS package, activating the ROS environment (``setup.bash``) is enough.

If this command works, you can go on with the next commands:

.. code-block:: console

   $ mvsim --version

You can get a list of available commands with:

.. code-block:: console

   $ mvsim --help
   mvsim v0.7.2: A lightweight multivehicle simulation environment.

   Available commands:
      mvsim launch <WORLD.xml>  Start a comm. server and simulates a world.
      mvsim server              Start a standalone communication server.
      mvsim node                List connected nodes, etc.
      mvsim topic               Inspect, publish, etc. topics.
      mvsim --version           Shows program version.
      mvsim --help              Shows this information.

   Or use `mvsim <COMMAND> --help` for further options


Command ``mvsim launch``
--------------------------

.. code-block:: console

   $ mvsim launch --help
   Usage: mvsim launch <WORLD_MODEL.xml> [options]

   Available options:
   --headless              Launch without GUI (e.g. suitable for dockerized envs.)
   --full-profiler         Enable full profiling (generates file with all timings)
   --realtime-factor <1.0> Run slower (<1) or faster (>1) than real time if !=1.0
   -v, --verbosity         Set verbosity level: DEBUG, INFO (default), WARN, ERROR


This can be used to launch the simulation of a world given the path to its XML definition file. Then you can interact with the robot(s) via keyboard, mouse, or joystick, or via the Python API.

If you want the simulator to communicate via ROS, you must launch the `ROS node <mvsim_node.html>`_ instead.


Command ``mvsim server``
--------------------------

.. code-block:: console

   $ mvsim server --help
   Usage: mvsim server

   Available options:
   -p 23700, --port 23700   Listen on given TCP port.
   -v, --verbosity      Set verbosity level: DEBUG, INFO (default), WARN, ERROR


MVSim uses a communication server, and then the simulation of the world happens in a client called "World". This command can be used to launch an independent, new server.
Normally you will not need to use this command unless you are developing a custom C++-based simulation based on the MVSim libraries.


Command ``mvsim node``
--------------------------

.. code-block:: console

   $ mvsim node --help
   Usage:

      mvsim node --help     Show this help
      mvsim node list       List all nodes connected to the server.

Request what MVSim network nodes are active and connected to the running server.


Command ``mvsim topic``
--------------------------

.. code-block:: console

   $ mvsim topic --help
   Usage:

      mvsim topic --help            Show this help
      mvsim topic list [--details]  List all advertised topics in the server
      mvsim topic echo <topicName>  Subscribe and print a topic
      mvsim topic hz <topicName>    Estimate topic publication rate (in Hz)

Can be used to list or inspect the publication of MVSim (not ROS!) topics with sensor and pose data.
These topics are accessible via the provided Python API, refer to examples: https://github.com/MRPT/mvsim/tree/develop/examples_python
