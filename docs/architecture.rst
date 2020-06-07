Architecture
=================================

The project comprises:

 - A C++ library: ``libmvsim``
 - A ROS1 node. It can be run standalone.
 - ``mvsim-server``: A standalone program to run the simulation and, optionally,
   displaying a GUI live view of the world, accept keyboard/mouse orders, etc. 
   It also uses ZMQ+protobuf as a communication system for user programs to 
   interact with the simulation (for example, from a C++ or Python program).

