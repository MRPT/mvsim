.. mvsim documentation master file, created by
   sphinx-quickstart on Sat May 30 11:36:10 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

MVSim: a lightweight MultiVehicle Simulator
=============================================

.. toctree::
   :maxdepth: 3
   :hidden:

   Home <self>
   install
   first-steps
   architecture
   world
   vehicles
   sensors
   demo_worlds
   teleoperation
   mvsim-cli
   mvsim_node
   physics
   extending
   bibliography
   C++ API <doxygen/html/modules.html#http://>

.. (JLBC note: Do not remove the #http:// above, it's the only way I found to allow that link to be included in the TOC).


Lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots.
It is tailored to analysis of vehicle dynamics, wheel-ground contact forces, and
accurate simulation of typical robot sensors.

This project includes C++ and Python libraries, the standalone CLI application ``mvsim``,
and ROS 1 and ROS 2 nodes, and it is licensed under the permissive 3-clause BSD License.

If you use MVSim in your work, feel free of citing 
`the MVSim SoftwareX paper <https://www.sciencedirect.com/science/article/pii/S2352711023001395>`_ 
(or the `ArXiV preprint <https://arxiv.org/abs/2302.11033>`_): 

.. dropdown:: Citation in BibTex format

    .. code-block:: bibtex
    
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


.. card:: :octicon:`download` Installing
    :link: installing
    :link-type: ref

    How to install or build from sources

.. card:: :octicon:`mortar-board` First steps
    :link: first-steps
    :link-type: ref

    Launch your first simulations and play around...

.. card:: :octicon:`gear` World XML format specifications
    :link: world-definition-docs
    :link-type: ref

    All the details about how to design simulated worlds and robots


.. figure:: https://mrpt.github.io/mvsim-models/anims/warehouse-demo-mvsim.gif
  :alt: MVSIM Simulating Jackal Robot in ROS2
  :target: first-steps.html


Documentation credits:

   - Borys Tymchenko `@spsancti <https://github.com/spsancti>`_
   - Jose Luis Blanco Claraco `@jlblancoc <https://github.com/jlblancoc>`_


.. card:: :octicon:`mark-github` GitHub repository
    :link: https://github.com/MRPT/mvsim

    Link to the main Git repository

