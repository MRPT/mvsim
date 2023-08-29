.. _world_simulable:

World XML definition of ``Simulable`` entities
-------------------------------------------------

Vehicles and obstacles blocks share common C++ ``mvsim::Simulable`` and
``mvsim::VisualObject`` interfaces that provide the common parameters below.

.. note::

   The following parameters can appear in either the {vehicle,block} class
   definitions or in a particular instantiation block, depending on whether you
   want parameters to be common to all instances or not, respectively.


Related to topic publication
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Under the ``<publish> </publish>`` tag group:

- **publish\_pose\_topic**: If provided, the pose of this object will be published as a topic with message type ``mvsim_msgs::Pose``.
- **publish\_pose\_period**: Period (in seconds) for the topic publication.

Example:

.. code-block:: xml

	<publish>
	  <publish_pose_topic>/r1/pose</publish_pose_topic>
	  <publish_pose_period>50e-3</publish_pose_period>
	</publish>
