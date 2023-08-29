.. _world_visual_object:

World XML definition of ``VisualObject``
--------------------------------------------

Vehicles and obstacles blocks share common C++ ``mvsim::Simulable`` and
``mvsim::VisualObject`` interfaces that provide the common parameters below.

.. note::

   The following parameters can appear in either the {vehicle,block} class
   definitions or in a particular instantiation block, depending on whether you
   want parameters to be common to all instances or not, respectively.


Related to visual aspect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Under the ``<visual> </visual>`` tag group:

- **model\_uri**: Path to 3D model file. Can be any file format supported by ASSIMP,
  like ``.dae``, ``.stl``, etc. If empty, the default visual aspect will be used.
- **model\_scale**: (Default=1.0) Scale to apply to the 3D model.
- **model\_offset_x**, **model\_offset_y** , **model\_offset_z**: (Default=0) Offset translation [meters].
- **model\_yaw**, **model\_pitch**, **model\_roll**: (Default=0) Optional model rotation [degrees].
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

