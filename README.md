MultiVehicle simulator (libmvsim) 
======================================
Lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots. 
Includes C++ library, standalone app and ROS node.

License: GNU General Public License version 3
Copyright (C) 2014 Jose Luis Blanco <jlblanco@ual.es> (University of Almeria) and collaborators

![screenshot](https://raw.githubusercontent.com/ual-arm-ros-pkg/multivehicle-simulator/master/docs/imgs/screenshot_scans_see_each_other.png "Screenshot 1")

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

Docs
----------
  * **The mvsim manual** is the main reference document. (Write me!!)
  * ROS tutorials (Write me!!)

Compiling: standalone
-----------------------
Requisites:
 * A decent C++ compiler!
 * MRPT (>=1.0.0 required; >=1.2.2 recommended): In Windows, build from sources or install precompiled binaries. 
 * Box2D: Will use an embedded copy if no system version is found.

In Ubuntu, this will install all requirements:

     sudo apt-get install libmrpt-dev libbox2d-dev

Compiling: ROS & catkin
------------------------
 * Clone this project into your catkin's workspace src folder.
 * Run catkin_make

Compiling: MOOS / OpenMORA
---------------------------
This package is already included in [OpenMORA](https://github.com/OpenMORA).
