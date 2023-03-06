[![mvsim](https://circleci.com/gh/MRPT/mvsim.svg?style=svg)](https://circleci.com/gh/MRPT/mvsim) [![Documentation Status](https://readthedocs.org/projects/mvsimulator/badge/?version=latest)](https://mvsimulator.readthedocs.io/en/latest/?badge=latest)
[![CI Linux](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml)
[![CI Check clang-format](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml)

MultiVehicle simulator (MVSIM)
======================================
Lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots.
It is tailored to analysis of vehicle dynamics, wheel-ground contact forces and accurate simulation of typical robot sensors (e.g. 2D and 3D lidars).

This package includes C++ libraries, standalone applications, and ROS 1 and ROS 2 nodes.

License: 3-clause BSD License
Copyright (C) 2014-2023 Jose Luis Blanco <jlblanco@ual.es> (University of Almeria) and collaborators

Please, refer to [the MVSim paper](https://arxiv.org/abs/2302.11033) for a gentle introduction
to the project architecture.
If you want to cite MVSim in your work, please use:

    @misc{mvsim,
      doi = {10.48550/ARXIV.2302.11033},
      url = {https://arxiv.org/abs/2302.11033},
      author = {Blanco-Claraco, José-Luis and Tymchenko, Borys and Mañas-Alvarez, Francisco José and Cañadas-Aránega, Fernando and López-Gázquez, Ángel and Moreno, José Carlos},  
      title = {MultiVehicle Simulator (MVSim): lightweight dynamics simulator for multiagents and mobile robotics research},  
      publisher = {arXiv},
      year = {2023}
    }


Installation
--------------------

See [installation documentation](https://mvsimulator.readthedocs.io/en/latest/install.html) for all the details and options. 

The easiest way to install if you already have ROS 1 or ROS 2 is:

    sudo apt install ros-$ROS_DISTRO-mvsim

Then jump to [next steps](https://mvsimulator.readthedocs.io/en/latest/first-steps.html) to see how to launch some of the demo worlds.


Demo videos
--------------------

![screenshot-demo](docs/imgs/mvsim-ros2-demo.gif)

[![MvSim intro](https://img.youtube.com/vi/xMUMjEG8xlk/0.jpg)](https://www.youtube.com/watch?v=xMUMjEG8xlk)


Build matrix status
--------------------

| Distro | Build dev | Build releases | Stable sync |
| ---    | ---       | ---            | ---         |
| ROS 1 Melodic (u18.04) | [![Build Status](https://build.ros.org/job/Mdev__mvsim__ubuntu_bionic_amd64/badge/icon)](https://build.ros.org/job/Mdev__mvsim__ubuntu_bionic_amd64/) | [![Build Status](https://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__mvsim__ubuntu_bionic_amd64__binary/badge/icon)](https://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__mvsim__ubuntu_bionic_amd64__binary/) | [![Version](https://img.shields.io/ros/v/melodic/mvsim)](https://index.ros.org/search/?term=mvsim) |
| ROS 1 Noetic (u20.04) | [![Build Status](https://build.ros.org/job/Ndev__mvsim__ubuntu_focal_amd64/badge/icon)](https://build.ros.org/job/Ndev__mvsim__ubuntu_focal_amd64/) |  [![Build Status](https://build.ros.org/job/Nbin_uF64__mvsim__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros.org/job/Nbin_uF64__mvsim__ubuntu_focal_amd64__binary/)  | [![Version](https://img.shields.io/ros/v/noetic/mvsim)](https://index.ros.org/search/?term=mvsim) |
| ROS 2 Foxy (u20.04) | [![Build Status](https://build.ros2.org/job/Fdev__mvsim__ubuntu_focal_amd64/badge/icon)](https://build.ros2.org/job/Fdev__mvsim__ubuntu_focal_amd64/) |  [![Build Status](https://build.ros2.org/job/Fbin_uF64__mvsim__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros2.org/job/Fbin_uF64__mvsim__ubuntu_focal_amd64__binary/) | [![Version](https://img.shields.io/ros/v/foxy/mvsim)](https://index.ros.org/search/?term=mvsim) |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/) |  [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mvsim)](https://index.ros.org/search/?term=mvsim) |
| ROS 2 Rolling (u22.04) | [![Build Status](https://build.ros2.org/job/Rdev__mvsim__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mvsim__ubuntu_jammy_amd64/) |  [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/)  | [![Version](https://img.shields.io/ros/v/rolling/mvsim)](https://index.ros.org/search/?term=mvsim) |



Docs
----------
  * [Main documentation site](https://mvsimulator.readthedocs.io/en/latest/)
  * https://wiki.ros.org/mvsim

If you clone this repository, remember to checkout the git submodules too:

    git clone https://github.com/MRPT/mvsim.git --recursive

Launch demos
--------------

See more on first steps [here](https://mvsimulator.readthedocs.io/en/latest/first-steps.html).

Standalone:

    mvsim launch mvsim_tutorial/demo_warehouse.world.xml
    mvsim launch mvsim_tutorial/demo_2robots.world.xml
    mvsim launch mvsim_tutorial/test_mesh.world.xml


ROS 1:

    roslaunch mvsim demo_depth_camera.launch

ROS 2:

    ros2 launch mvsim demo_warehouse.launch.py
    ros2 launch mvsim demo_depth_camera.launch.py

Main features
--------------
  * Lightweight in memory, CPU and library requirements.
  * Fully configurable via `.xml` "world" files.
  * Headless mode, suitable for dockerized environments.
  * World maps:
    * Occupancy gridmaps: input as images or MRPT binary maps (from icp-slam, rbpf-slam, etc.)
    * Elevation meshes.
  * Vehicle models:
    * Differential driven (2 & 4 wheel drive).
    * Ackermann steering (kinematic & dynamic steering, different mechanical drive models).
    * Ackermann steering with mechanical differentials of full grade.
  * Sensors:
    * 2D and 3D Lidars: Robots see each other, their own bodies, etc.
    * RGB cameras
    * Depth cameras
  * Interface to vehicles: Custom Python interface, or ROS. Choose among:
    * Raw access to forces and motor torques.
    * Twist commands (using internal controllers).

