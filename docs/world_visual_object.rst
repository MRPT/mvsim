.. _world_visual_object:

World XML definition of ``VisualObject``
--------------------------------------------

Vehicles and obstacles blocks share common C++ ``mvsim::Simulable`` and
``mvsim::VisualObject`` interfaces that provide the common parameters below.

.. note::

   The following parameters can appear in either the {vehicle,block} class
   definitions or in a particular instantiation block, depending on whether you
   want parameters to be common to all instances or not, respectively.

.. dropdown:: Repository of reusable 3D models

   A list of 3D models, readily usable within MVSim, is maintained in `this Git repository <https://github.com/MRPT/mvsim-models>`_,
   which is accessible via HTTPS through `this index <https://mrpt.github.io/mvsim-models/>`_. Refer to example worlds
   for case of use of these objects and vehicles.


Related to visual aspect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Under the ``<visual> </visual>`` tag group:

- **model_uri**: Path to 3D model file. Can be any file format supported by ASSIMP,
  like ``.dae``, ``.stl``, etc. If empty, the default visual aspect will be used.
- **model_scale**: (Default=1.0) Scale to apply to the 3D model.
- **model_offset_x**, **model_offset_y** , **model_offset_z**: (Default=0) Offset translation [meters].
- **model_yaw**, **model_pitch**, **model_roll**: (Default=0) Optional model rotation [degrees].
- **model_color**: (Default: white) For colorless files (e.g. ``*.stl``), the color to use (see :ref:`color formatting <world_value_parsing>`).
- **model_cull_faces**: (Default=``NONE``) Can be one of ``NONE | BACK | FRONT``.
- **model_split_size**: (Default=0.0, disabled) Only required for semi-transparent meshes. Defines the size [meters] of auxiliary voxels used to split triangles and help sorting them by depth for correct rendering.
- **show_bounding_box**: (Default=``false``) Initial visibility of the object bounding box.

Example:

.. code-block:: xml

	<visual>
	  <model_uri>robot.obj</model_uri>
	  <model_scale>1.0</model_scale>
	  <model_offset_x>0.0</model_offset_x>
	  <model_offset_y>0.0</model_offset_y>
	  <model_offset_z>0.0</model_offset_z>
	</visual>

