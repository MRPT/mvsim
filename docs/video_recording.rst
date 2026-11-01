.. _video-recording:

Video recording
=================

MVSim can record videos of its 3D view. Frames are taken in **simulation
time**: one every ``1/fps`` seconds of simulated time, and the simulation waits
for each one, as it does for camera sensors. Hence, videos play at real speed
and never drop frames, whether the simulation runs faster or slower than real
time, in the GUI or in headless mode.

Frames are rendered off-screen at a fixed resolution (independent of the
window size, and without the GUI panels), and encoded by an ``ffmpeg`` process,
which must be installed (e.g. ``sudo apt install ffmpeg``). Alternatively, if
the output file name has a printf-like pattern (e.g. ``frames/img_%06d.png``),
each frame is saved as an image file.

A recording can be started and stopped from:

* The **GUI**: the record button in the status bar, or the ``File`` menu
  ("Record video", "Record video as...", "Stop recording").
* The **world file**: a ``<video_recording>`` block, which also sets the
  default options for the other methods.
* **ZMQ services**, e.g. from Python scripts.
* **C++**, with ``World::videoRecorder()``.

|

World file options
---------------------

.. code-block:: xml

    <!-- enabled="true": start recording at <start_time> -->
    <video_recording enabled="true">
      <output_file>truck.mp4</output_file>  <!-- default: mvsim_video_<date>_<time>.mp4 -->
      <fps>30</fps>
      <width>1920</width>
      <height>1080</height>
      <start_time>2.0</start_time>   <!-- [s] simulation time -->
      <duration>10.0</duration>      <!-- [s] of simulated time (0: until stopped) -->
      <close_simulator_when_done>true</close_simulator_when_done>
      <ffmpeg_options>-c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p</ffmpeg_options>

      <!-- Camera: "gui" (default) or "orbit" -->
      <camera>orbit</camera>
      <follow_vehicle>r1</follow_vehicle>
      <follow_heading>true</follow_heading>  <!-- azimuth relative to the vehicle heading -->
      <cam_point_to>1.5 0 1.0</cam_point_to>  <!-- in the vehicle frame, if following one -->
      <cam_distance>12</cam_distance>
      <cam_azimuth>-120</cam_azimuth>    <!-- [deg] -->
      <cam_elevation>15</cam_elevation>  <!-- [deg] -->
      <fov_deg>50</fov_deg>
      <ortho>false</ortho>
    </video_recording>

With ``<camera>gui</camera>``, the video shows the GUI view, following the
user's camera motions. In headless mode, that camera is defined by the
``<gui>`` options (``cam_distance``, ``cam_azimuth``, ``follow_vehicle``...).
With ``<camera>orbit</camera>``, the camera is defined by the parameters in the
``<video_recording>`` block, e.g. to track a vehicle from a fixed angle
relative to its heading.

For unattended recordings, combine ``enabled="true"``, a ``duration`` and
``close_simulator_when_done`` with headless mode and the fastest possible
simulation speed:

.. code-block:: bash

    mvsim launch my_world.world.xml --headless --realtime-factor 0

Notes:

* The simulation time step should not be longer than the frame period
  (``1/fps``). Otherwise, frames are repeated to keep the video timing.
* While recording, the simulation may run slower than real time, since it
  waits for each frame (and for the encoder, if it is slower).
* Video codecs require even sizes: odd widths or heights are rounded down.
* Stopping (or closing the simulator) completes the video file.

|

ZMQ services
---------------

==============================  =========================================  ===============================================
Service                         Request / Answer                           Description
==============================  =========================================  ===============================================
``start_video_recording``       ``SrvStartVideoRecording`` /               Starts a recording. Unset fields take the
                                ``SrvStartVideoRecordingAnswer``           ``<video_recording>`` values, or the defaults.
``stop_video_recording``        ``SrvStopVideoRecording`` /                Stops it, once the file is complete. Returns
                                ``SrvStopVideoRecordingAnswer``            its number of frames and duration.
``get_video_recording_status``  ``SrvGetVideoRecordingStatus`` /           Whether recording, frames written so far,
                                ``SrvGetVideoRecordingStatusAnswer``       duration, and the last encoder error.
==============================  =========================================  ===============================================

See the Python example
`examples_python/record-video.py <https://github.com/MRPT/mvsim/blob/develop/examples_python/record-video.py>`_:

.. code-block:: bash

    mvsim launch mvsim_tutorial/demo_suspension.world.xml
    # In another terminal:
    python3 examples_python/record-video.py --output truck.mp4 --duration 10 --follow r1

|

From C++
-----------

.. code-block:: cpp

    #include <mvsim/World.h>

    mvsim::VideoRecorder::Options opts;
    opts.output_file = "video.mp4";
    opts.duration = 5.0;
    world.videoRecorder().start(opts);  // throws if ffmpeg can not be launched
    // ... run the simulation ...
    const auto status = world.videoRecorder().stop();
