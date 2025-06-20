[![mvsim](https://circleci.com/gh/MRPT/mvsim.svg?style=svg)](https://circleci.com/gh/MRPT/mvsim) [![Documentation Status](https://readthedocs.org/projects/mvsimulator/badge/?version=latest)](https://mvsimulator.readthedocs.io/en/latest/?badge=latest)
[![CI Linux](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml)
[![CI Check clang-format](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml)

MultiVehicle simulator (MVSim)
======================================
Lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots.
It is tailored to analysis of vehicle dynamics, wheel-ground contact forces and accurate simulation of typical robot sensors (e.g. 2D and 3D lidars).

This package includes C++ libraries, standalone applications, and ROS 1 and ROS 2 nodes.

License: 3-clause BSD License
Copyright (C) 2014-2025 Jose Luis Blanco <jlblanco@ual.es> (University of Almeria) and collaborators

Please, refer to [the MVSim SoftwareX paper](https://www.sciencedirect.com/science/article/pii/S2352711023001395) (or the [ArXiV preprint](https://arxiv.org/abs/2302.11033))
for a gentle introduction to the project architecture.
If you want to cite MVSim in your work, please use:

    @article{blanco2023mvsim,
      title = {MultiVehicle Simulator (MVSim): Lightweight dynamics simulator for multiagents and mobile robotics research},
      journal = {SoftwareX},
      volume = {23},
      pages = {101443},
      year = {2023},
      issn = {2352-7110},
      doi = {https://doi.org/10.1016/j.softx.2023.101443},
      url = {https://www.sciencedirect.com/science/article/pii/S2352711023001395},
      author = {José-Luis Blanco-Claraco and Borys Tymchenko and Francisco José Mañas-Alvarez and Fernando Cañadas-Aránega and Ángel López-Gázquez and José Carlos Moreno}
    }

ROSCon talk
------------------
Spanish talk with English slides and subtitles ([slides here](https://docs.google.com/presentation/d/1jX8t1r82vp8MIQP5u1t9bVTtG0XgmjDTtLI7z3Yamzc/edit?usp=sharing)):

[![MvSim ROSCon talk](https://img.youtube.com/vi/WNBqH6SWlRQ/0.jpg)](https://www.youtube.com/watch?v=WNBqH6SWlRQ)


Installation
--------------------

See [installation documentation](https://mvsimulator.readthedocs.io/en/latest/install.html) for all the details and options. 

The easiest way to install if you already have ROS 1 or ROS 2 is:

    sudo apt install ros-$ROS_DISTRO-mvsim

Then jump to [next steps](https://mvsimulator.readthedocs.io/en/latest/first-steps.html) to see how to launch some of the demo worlds.


Demo videos
--------------------

https://github.com/user-attachments/assets/766db164-2d16-44f4-acbf-2f15b73c1ab3

![screenshot-demo](docs/imgs/mvsim-ros2-demo.gif)

[![MvSim intro](https://img.youtube.com/vi/xMUMjEG8xlk/0.jpg)](https://www.youtube.com/watch?v=xMUMjEG8xlk)


Build matrix status
--------------------

| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mvsim__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mvsim__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |


| EOL distro | Stable version |
| ---    | ---       |
| ROS 1 Melodic (u18.04) | [![Version](https://img.shields.io/ros/v/melodic/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 1 Noetic (u20.04) | [![Version](https://img.shields.io/ros/v/noetic/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Foxy (u20.04) | [![Version](https://img.shields.io/ros/v/foxy/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Iron (u22.04) | [![Version](https://img.shields.io/ros/v/iron/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |


Docs
----------
  * [Main documentation site](https://mvsimulator.readthedocs.io/en/latest/)

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

