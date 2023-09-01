.. _world_remote-resources:

Remote resources notation
---------------------------

When defining a world, it is common to use several **large files** for textures or 3D meshes.
MVSim allows users to store those files in **external web servers** accessible via HTTP/HTTPS and
**download them on the fly** when loading a world XML file.
Optionally, several such files may be compressed in a single **ZIP file** which will be
downloaded and decompressed automatically.

.. note:: All remote files are downloaded and then stored in a local cache,
    kept by default in ``$HOME/.cache/mvsim-storage``. Remove the contents herein
    if you want to force redownloading some files.

The notation is used whenever a file needs to be referenced in any parameter
within a :ref:`world definition file <world-definition-docs>`:

- **Remote server, individual files**: The resource must begin with ``http://`` or ``https://``.
  Example:

  .. code-block:: xml

    <texture_image>https://mrpt.github.io/mvsim-models/textures-cgbookcase/Pebbles02_512_BaseColor.png</texture_image>

- **Remote server, a file within a ZIP file**: The resource must begin with ``http://`` or ``https://``
  and it must point to the ZIP file, then append a ``/``, then the local path of the file within the ZIP.
  Note that the whole ZIP is downloaded to the local cache and extracted, so accessing several files in
  the same ZIP is more efficient.
  Example:

  .. code-block:: xml

    <model_uri>https://mrpt.github.io/mvsim-models/rock01.zip/rock01/FallingRock01.dae</model_uri>

- **Local filesystem**: The resource may *optionally* begin with ``file://``, or just
  specify the local or global path to the file. Here one can also refer to an individual file or
  to a file within a ZIP file.
  Examples:

  .. code-block:: xml

    <model_uri>${MVSIM_CURRENT_FILE_DIRECTORY}/../models/simple_camera.dae</model_uri>
    ...
    <model_uri>${MY_REPO_DIR}/models.zip/vehicle.dae</model_uri>

Additionally, in combination with all the options above, and while reading the world XML file,
the :ref:`MVSim XML parser <world_xml_parser>` resolves environment variables, mathematical expressions, and
the output of external programs, to make your XML files more reusable, flexible, and readable.


.. note:: The features described above are implemented in the C++ class 
    `mvsim::RemoteResourcesManager <https://github.com/MRPT/mvsim/blob/develop/modules/simulator/src/RemoteResourcesManager.cpp>`_.
