MultiVehicle 2D simulator (libmv2dsim) 
======================================
Lightweight, realistic dynamical simulator for 2D vehicles/robots. 
Includes C++ library, standalone app and ROS node.

License: GNU General Public License version 3
Author: Jose Luis Blanco <jlblanco@ual.es> (University of Almeria)

![screenshot](https://raw.githubusercontent.com/ual-arm-ros-pkg/multivehicle_2d_simulator/master/docs/imgs/screenshot_scans_see_each_other.png "Screenshot 1")


Main features
--------------
  * Fully configurable via `.xml` "world" files and (TODO) extensible via user Python scripts.
  * World maps:
    * Occupancy gridmaps: input as images or MRPT binary maps (from icp-slam, rbpf-slam, etc.)
  * Vehicle models: 
    * Differential driven. (TODO: 2 & 4 wheels)
    * Ackermann steering. (TODO: kinematic./dyn. steer wheel control)
  * Sensors: 
    * Laser scanners: Robots see each other, their own bodies, etc.
  * Interface to vehicles: Choose among: 
    * Raw access to forces and torques.
    * Twist commands (use internal controller).
  * Lightweight in memory, CPU and library requirements.

Compiling
----------
Requisites:
 * A decent C++ compiler!
 * MRPT (>=1.0.0): In Windows, build from sources or install precompiled binaries. 
 * Box2D: Will use an embedded copy if no system version is found.

In Ubuntu, this will install all requirements:

     sudo apt-get install libmrpt-dev libbox2d-dev


