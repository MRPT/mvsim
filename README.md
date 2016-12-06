[![Build Status](https://travis-ci.org/ual-arm-ros-pkg/mvsim.svg?branch=master)](https://travis-ci.org/ual-arm-ros-pkg/mvsim)

MultiVehicle simulator (libmvsim) 
======================================
Lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots. 
It is tailored to analysis of vehicle dynamics, wheel-ground contact forces and accurate simulation of typical robot sensors (e.g. laser scanners).

This package includes the C++ library `mvsim`, a standalone app and a ROS node.

License: GNU General Public License version 3
Copyright (C) 2014 Jose Luis Blanco <jlblanco@ual.es> (University of Almeria) and collaborators

![screenshot](https://raw.githubusercontent.com/ual-arm-ros-pkg/mvsim/master/docs/imgs/screenshot_scans_see_each_other.png "Screenshot 1")

Docs
----------
  * (TO-DO) **The mvsim manual** is the main reference document. (Write me!!)
  * (TO-DO) ROS tutorials (Write me!!)
  * http://wiki.ros.org/mvsim

Main features
--------------
  * Lightweight in memory, CPU and library requirements.
  * Fully configurable via `.xml` "world" files.
  * World maps:
    * Occupancy gridmaps: input as images or MRPT binary maps (from icp-slam, rbpf-slam, etc.)
    * Elevation meshes.
  * Vehicle models: 
    * Differential driven (2 & 4 wheel drive).
    * Ackermann steering (kinematic & dynamic steering, different mechanical drive models).
  * Sensors: 
    * Laser scanners: Robots see each other, their own bodies, etc.
  * Interface to vehicles: Choose among:
    * Raw access to forces and motor torques.
    * Twist commands (using internal controllers).


Compiling: standalone
-----------------------
Requisites:
 * A decent C++ compiler!
 * MRPT (>=1.0.0 required; >=1.2.2 recommended): In Windows, build from sources or install precompiled binaries. 
 * Box2D: Will use an embedded copy if no system version is found.

In Ubuntu, this will install all requirements:

     sudo apt-get install libmrpt-dev libbox2d-dev

ROS: Compiling & usage
------------------------
 * Install: 
     sudo apt-get install ros-$ROS_DISTRO-mvsim

 * Usage: See docs and tutorials in http://wiki.ros.org/mvsim 

Compiling: MOOS / OpenMORA
---------------------------
This package is already included in [OpenMORA](https://github.com/OpenMORA).
