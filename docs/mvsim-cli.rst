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
      mvsim slice <WORLD.xml>   Export a 2D slice of the world obstacles to a .txt file.
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
   -p, --port <23700>      TCP port of the communications server. If not given and the
                           default port is in use, the next free one is used. The
                           environment variable MVSIM_SERVER_PORT also sets it.
   --full-profiler         Enable full profiling (generates file with all timings)
   --realtime-factor <1.0> Run slower (<1) or faster (>1) than real time if !=1.0,
                           or as fast as possible if <=0
   -v, --verbosity         Set verbosity level: DEBUG, INFO (default), WARN, ERROR


This can be used to launch the simulation of a world given the path to its XML definition file. Then you can interact with the robot(s) via keyboard, mouse, or joystick, or via the Python API.

If you want the simulator to communicate via ROS, you must launch the `ROS node <mvsim_node.html>`_ instead.

The communications server listens at TCP port 23700 by default. If that port is used by another
program, ``mvsim launch`` picks the next free port (up to 23709) and prints a warning with its number.
Set the environment variable ``MVSIM_SERVER_PORT`` to that number (or pass ``--port`` to ``mvsim topic``
and ``mvsim node``) so other tools and the Python clients connect to the right server. If ``--port``
or ``MVSIM_SERVER_PORT`` is given, that exact port is required and an error is reported if it is not available.


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


Command ``mvsim slice``
--------------------------

.. code-block:: console

   $ mvsim slice --help
   Usage: mvsim slice <WORLD_MODEL.xml> -o <OUTPUT.txt> [options]

   Available options:
   -o, --output <FILE>          Output .txt file (mandatory).
   --slice-z <0.5>              Height of the slice [m].
   --slice-z-max <z>            If given, the cells of all geometry between
                                --slice-z and this height [m] are reported.
   --slice-resolution <0.05>    Cell size [m].
   --slice-static-only          Skip blocks that are not <static> (movable).
   -v, --verbosity              Set verbosity level: DEBUG, INFO (default), WARN, ERROR

Loads a world (without simulating it) and writes the occupied cells of a 2D grid where
its geometry crosses a given height, or a height band, as 3D sensors would see it at t=0:
the actual triangles of 3D models, walls with their doors and windows, primitives, point
clouds and occupancy grid elements. Vehicles, actors and animated blocks are skipped,
since they move. The output has one ``x y z`` line per occupied cell center, e.g. to build
a static obstacle map for path planning. For example, as an ``mp2p_icp`` metric map:

.. code-block:: console

   $ mvsim slice demo_warehouse.world.xml -o obstacles.txt --slice-z 0.1 --slice-z-max 1.5
   $ txt2mm -i obstacles.txt -o obstacles.mm -f xyz

Random expressions in the world file (e.g. ``$f{rand()}``) give the same values as in a
simulation only if the world fixes the random seed first, e.g. with
``<variable name="dummy" value="$f{randomize(1)}"></variable>``.
