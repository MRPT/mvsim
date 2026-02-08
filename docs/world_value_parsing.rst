.. _world_value_parsing:

Parameters entries and expected formatting
---------------------------------------------------

MVSim uses a custom parameter parsing system defined in ``TParameterDefinitions`` 
and ``TParamEntry`` classes. This system extends standard C ``sscanf`` format 
specifiers with additional types commonly used in robotics and simulation.

Parameters can be defined in XML as either:

- **XML attributes**: ``<block mass="50.0" static="true"/>``
- **XML child elements**: ``<block><mass>50.0</mass><static>true</static></block>``

Both forms are equivalent and parsed using the same system.

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
     - Expects ``true``, ``false``, ``1``, or ``0`` (case-insensitive)

Special handling
^^^^^^^^^^^^^^^^^^

NaN values
~~~~~~~~~~

Some parameters use NaN (Not a Number) as a sentinel value to indicate "not set" 
or "use default". For example:

- Block ``visual_scale``: NaN means use the class-defined ``model_scale``
- Block ``zmin`` / ``zmax``: NaN means auto-calculate from visual model

When parameters have default NaN values, omitting them in XML leaves them as NaN, 
which triggers the appropriate fallback behavior.
