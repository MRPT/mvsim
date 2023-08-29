Sensors
===================

This page describes the predefined sensors and their parameters.

Sensors are defined with **<sensor>** tag. It has attributes *type* and
*name*.

At the moment, only laser scanner sensor is implemented, its type is
*laser*. Subtags are:

-  **<pose>** - an MRPT CPose3D string value

-  **<fov\_degrees>** - FOV of the laser scanner

-  **<sensor\_period>** - period in seconds when sensor sends updates

-  **<nrays>** - laser scanner rays per FOV

-  **<range\_std\_noise>** - standard deviation of noise in distance
   measurements

-  **<angle\_std\_noise\_deg>** - standatd deviation of noise in angles
   of rays

-  **<bodies\_visible>** - boolean flag to see other robots or not


