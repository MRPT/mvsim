.. _world_value_parsing:

Parameters entries and expected formatting
---------------------------------------------------

Describe the formats in TParameterDefinitions and TParamEntry.

All those defined by standard ``sscanf``, plus these extensions:

.. list-table:: MVSim extensions to ``sscanf`` formats
   :widths: 20 30 50
   :header-rows: 1

   * - Format specifier
     - C++ pointer to...
     - Expected format
   * - ``%lf_deg``
     - ``double``
     - A number, in degrees.
   * - ``%s``
     - ``std::string``
     - A string. Whitespace will be trimmed.
   * - ``%color``
     - ``mrpt::img::TColor``
     - A color like ``#RRGGBB[AA]`` ([00-FF] each)
   * - ``%pose2d``
     - ``mrpt::poses::CPose2D``
     - Expects ``X Y YAW_DEG``
   * - ``%pose2d_ptr3d``
     - ``mrpt::poses::CPose3D``
     - Expects ``X Y YAW_DEG``
   * - ``%pose3d``
     - ``mrpt::poses::CPose3D``
     - Expects ``X Y Z YAW_DEG PITCH_DEG ROLL_DEG``
   * - ``%point3d``
     - ``mrpt::math::TPoint3D``
     - Expects ``X Y Z YAW_DEG PITCH_DEG ROLL_DEG``
   * - ``%bool``
     - ``bool``
     - Expects ``X Y [Z]`` (default ``Z=0``)
