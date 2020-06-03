Features
===========

- Lightweight in memory, CPU and library requirements.

- Fully configurable via .xml "world" files.

- World maps:

  - Occupancy gridmaps: input as images or MRPT binary maps (from icp-slam, rbpf-slam, etc.)

  - Elevation meshes.

- Vehicle models:

  - Differential driven (2 \& 4 wheel drive).

  - Ackermann steering (kinematic \& dynamic steering, different mechanical drive models).

- Sensors:

  - 2D lidar scanners: Robots see each other, their own bodies, etc.

- Interface to vehicles: Choose among:

  - Raw access to forces and motor torques.

  - Twist commands (using internal controllers).
