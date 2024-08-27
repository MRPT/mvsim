^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mvsim
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.0 (2024-08-28)
-------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Merge pull request `#52 <https://github.com/MRPT/mvsim/issues/52>`_ from finani/fix-fake-amcl
  Fix fake amcl topic (QoS Durability)
* Update outdated copyright years in source files
* Merge pull request `#51 <https://github.com/MRPT/mvsim/issues/51>`_ from finani/nav2-compatible
  Nav2 compatible
* Update codebase to new clang-format style
* Merge pull request `#48 <https://github.com/MRPT/mvsim/issues/48>`_ from finani/refactoring-node
  Refactoring node
* Merge pull request `#45 <https://github.com/MRPT/mvsim/issues/45>`_ from finani/develop
  Add tf pub for each robot (support namespace)
* Merge pull request `#43 <https://github.com/MRPT/mvsim/issues/43>`_ from kr-jschoi/fix-bug
  Fix namespace switching bug between mvsim.chassis_shape and mvsim.chassis_shape.wheel3 in /chassis_markers
* Contributors: Inhwan Wee, Jose Luis Blanco-Claraco, finani, kr-jschoi

0.9.4 (2024-05-08)
------------------
* Better detection of "in collision" flag.
* Update to clang-format-14
* Upgrade Joystick API so it works correctly with an arbitrary number of axes
* ROS nodes: add collision state publishers for each vehicle
* remove dead code
* Contributors: Jose Luis Blanco-Claraco

0.9.3 (2024-04-24)
------------------
* Fix usage of (new explicit) TPoint3D constructors
* Contributors: Jose Luis Blanco-Claraco

0.9.2 (2024-03-21)
------------------
* BUG FIX: 3D lidars should not 'see' XYZ corners of wheels
* BUG FIX: gridmaps were published at a too high rate in the ROS 2 node
* remove dead code
* update header version
* Contributors: Jose Luis Blanco-Claraco

0.9.1 (2024-03-05)
------------------
* Fix use of mrpt bridge to publish XYZIRT point clouds too for ROS 1
* Contributors: Jose Luis Blanco-Claraco

0.9.0 (2024-03-02)
------------------
* Do not publish tf world->map
* Expose do_fake_localization as ROS 2 launch file argument
* fix build with older mrpt
* 3D Lidar: also generate "ring" ID per point
* Contributors: Jose Luis Blanco-Claraco

0.8.3 (2023-12-05)
------------------
* Generate ground truth trajectory files in the TUM format
* ROS nodes: add a timeout for cmd_vel commands, so the robots stop if input messages stop
* Add rviz_plugin_tutorials dependency for ROS1
* Contributors: Jose Luis Blanco-Claraco, Michael Grupp

0.8.2 (2023-10-06)
------------------
* 3D LIDARs: Bilinear interpolation (when it makes sense) to obtain much smoother point cloud simulations
* Add missing build dep on python3-pip.
  This was triggering errors on ROS 1 build farm dev builds.
* Contributors: Jose Luis Blanco-Claraco

0.8.1 (2023-09-07)
------------------
* Move the rawlog-generation option to the World global options instead of sensor-wise.
* Create CITATION.cff
* helios 32fov70 sensor.xml: Fix missing MVSIM_CURRENT_FILE_DIRECTORY tag
* Fix crash in edge case with world file path in the current directory
* Contributors: Jose Luis Blanco-Claraco

0.8.0 (2023-09-02)
------------------
* Recursive "include"s are now possible.
* All vehicle and sensor definitions are now exposed in public directory 'definitions' and are safe to be included from user worlds
* ROS warehouse demos: fix wrong camera topicn ame in rviz
* Add missing ROS 2 launch demo for greenhouse world
* Add new variable: MVSIM_CURRENT_FILE_DIRECTORY
* BUGFIX: In parseVars() in the XML parser
* Debugging feature: MVSIM_VERBOSE_PARSE now also traces <variable> definitions
* Contributors: Jose Luis Blanco-Claraco

0.7.4 (2023-08-30)
------------------
* Add new LIDAR 3D models: Helios 32.
* Removed LIDAR3D fbo_nrows parameter, automatically computed now from geometry solutions.
* More optimal simulation of asymmetric 3D lidars.
* Progress with RTD documentation.
* Add proper bibliography; fix all docs warnings
* ROS 2 warehouse demo: show 2D lidar in RVIZ too; add headless launch argument
* New GUI editor feature: move sensor poses
* Contributors: Jose Luis Blanco-Claraco

0.7.3 (2023-08-23)
------------------
* Update python module install method to pip (fixes deprecation)
* Improved docs
* Contributors: Jose Luis Blanco-Claraco

0.7.2 (2023-08-12)
------------------
* Joystick driving: added support for direct driving the vehicles with a joystick.
* Fix rviz for ros1 demo
* Better docs and more modern RST style.
* More shadow tuning parameters.
* IMU sensor now reads real vehicle linear acceleration.
* Contributors: Jose Luis Blanco-Claraco

0.7.1 (2023-06-11)
------------------
* ROS node: fix potential race condition creating publisher for highrate sensors
* Add IMU sensors
* New property <visual enabled='true|false'>
* lidar2d xml: add option "sensor_custom_visual"
* FIX: Crash if launching an empty world
* Trigger an error if using use_sim_time to avoid mistakes
* Add new (fake) controller: twist_ideal
* ROS 1: don't enforce /use_sim_time
* Add ROS 2 launch for the 2-robot demo
* Fix cmake leftover
* Contributors: Jose Luis Blanco-Claraco

0.7.0 (2023-04-21)
------------------
* Automatic detection of collision shapes
* Add turtlebot world demo
* Rely on custom build of Box2D to increase the maximum polygon vertices count (8 to 12).
* fix inconsistent use_sim_time value for ROS 2 (it should be false)
* Expose shadow rendering parameters in the XML world file
* Abort simulation on exceptions in headless mode too
* ROS 1 and 2 nodes now quit cleanly and quickly with SIGINT
* Add ros2 launch for turtlebot world demo
* Automatic determination of zmin/zmax for blocks if not explicitly set in XML
* Force c++17 for python module
* Exit simulator on exceptions in GUI-related threads
* More automated testing
* Add unit tests in C++ too
* Refactor collision shape determination
* New UI checkbox: show collision shapes
* Allow simply geometry definitions without external 3D model file for "blocks"
* Light options are now under <light> XML tag.
* Largest default physics simulation timestep changed from 50ms to 5ms
* GUI: change light direction
* Add GUI checkbox to enable/disable shadows
* Add rplidar A2 sensor model
* Contributors: Jose Luis Blanco-Claraco

0.6.1 (2023-03-04)
------------------
* New XML parameters to enable and tune shadowmap generation
* Use finer timestep for prevent wrong simulation of ramp sliding
* Fix code notation
* Temporary workaround to GH CI problem
* Contributors: Jose Luis Blanco-Claraco

0.6.0 (2023-02-26)
------------------
* Support for SkyBox rendering (requires MRPT >=2.7.0)
* More camera options in world.xml files (initial azimuth, elevation, etc.)
* Terrain elevation models now support repeated textures (requires MRPT >=2.7.0)
* Faster 3D Lidar rendering (Requires MRPT >=2.7.0)
* Add Ouster OS1 sensor file
* Fix default friction coefficients; draw motor torques too
* More accurate Velodyne simulation based on sensor_rpm parameter
* Clearer code and code style conventions
* Add "<static>" XML tag for large, static world objects
* Support for XML tag <if ...>
* Refactor xml parser as a registry of tag->function
* Examples renamed for conciseness: 'mvsim_demo\_*' to 'demo\_*'
* Added a "greenhouse" example world
* Wheels: allow linked-yaw-objects in vehicle viz
* Support several <visual> tags in custom visualization models
* pybind11 sources simplification.
  Simplify into one single source tree with conditional compilation for different pybind versions.
* Emit clearer warnings and earlier detection of wrong bounding boxes
* Add reference to (preprint) paper
* Controllers: Made threadsafe
* Contributors: Fernando Cañadas, Jose Luis Blanco-Claraco
* BUGFIX: program did not quit if using a non-existing launch file.
* BUGFIX: unneeded friction coefficient for chassis body
* BUGFIX: bbox for compound vehicle models
* BUGFIX: <for> loops ignored more than one inner tag
* BUGFIX: Add epsilon value for bbox determination in 3D models

0.5.2 (2023-01-27)
------------------
* FIX build farm errors in armhf builds
* Contributors: Jose Luis Blanco-Claraco

0.5.1 (2023-01-26)
------------------
* FIX: Stuck unit test runs in armhf build farms
* Add demo with a large number of robots (100) in a simple setup
* Support <for> loops in world definition files
* Contributors: Jose Luis Blanco-Claraco

0.5.0 (2023-01-09)
------------------
* New warehouse demo world file, including ros2 launch.
* New feature: download models from remote servers.
* Add 3D Lidar sensor.
* Add support for headless simulations (mvsim launch --headless), suitable for running inside docker containers
* New world element: vertical planes.
* Add <for /> loops in XML world files
* Support for formulas in XML files via  exprtk expressions
* Fix naming convention; fix warnings
* Move to clang-format-11
* More consistent class member naming convention
* Add simple Velodyne sensor DAE model
* Add wget as build and runtime dep
* PubSub system: Implemented the feature to notify subscribed clients about a new publisher for a topic
* New 3D model: pioneer3
* ROS: Add build and test dep python3-protobuf
* Added unit tests
* mvsim cli: add the --realtime-factor flag
* more topic echo types
* publish 2D lidar observations as custom protobuf msgs too
* Add new protobuf msg type ObservationLidar2D.proto
* add shutdown service
* Fixed Python topic subscription and parsing
* avoid potential crash during shutdown
* FIX: Timelogger verbosity level is now copied from the main World object.
* BUGFIX: Lidar sensors with ignore_parent_body=true should neither see the wheels
* Tune PID parameters of examples
* Refactoring and simplification of mutexes
* cache GetServiceInfoRequest() calls (more efficient service calls in comms::Client)
* factorize World services into its own .cpp file for clarity
* Contributors: Jose Luis Blanco-Claraco

0.4.3 (2022-11-14)
------------------
* add names to gl objects (useful to debug)
* configurable gui open timeout
* GUI: new button Editor->Export 3Dscene file
* Add a warning if using a physics timestep is incompatible with the sensor periods
* FIX: more accurate simulation of timesteps; more mutexes
* automatic determination of timestep; FIX bug: constant offset in simul time
* fix FBO api for mrpt 2.5.6
* fix build against mrpt 2.5.6; fix infinite loop while hitting quit button
* refactor TCLAP objects to avoid global object initialization fiasco crashes
* handle old and new versions of tf_geometry_msgs
* mvsim cli new command: topic hz
* Fix ROS2 obsolete header
* fix build for u18.04
* fix for older zmq versions
* fix newest zmq deprecated warnings
* FIX: DAE file error to load on newer version of assimp
* provide cmake config types for gcc sanitizers
* fix zmq API in bionic
* avoid zmq_message_t ctor; more parallel threads for ZMQ
* make the get_pose() service to answer immediately without needing to acquire the main simulation mutex
* fix escaping
* more portable makefile comparison for libbox2d version
* d/rules: auto detection of system libbox2d version
* Contributors: Jose Luis Blanco-Claraco

0.4.2 (2022-10-19)
------------------
* Disable Python wrappers for python <3.8
* Honor DESTDIR when building and installing
* Honor cli-provided PYTHON_INSTALL_DIRECTORY via cmake flags
* Fix protobuf-generated broken Python3 imports (using protoletariat)
* Add new WorldElement type: pointcloud
* Add Python3 example for teleop twist
* Contributors: Jose Luis Blanco-Claraco

0.4.1 (2022-10-13)
------------------
* Add more documentation, demo files, and screenshots
* Support animations from keyframe list for blocks and vehicles
* Refactor common xml params in Simulable interface
* Support PARENT_NAME usage in sensor definition files; add "<publish>" tags to tutorial sensors
* 2D lidar sensor: new XML parameter maxRange
* change threshold to decimate sensors preview subwindows
* BUGFIX: Uninitialized quaternion in rviz marker (Closes `#14 <https://github.com/MRPT/mvsim/issues/14>`_)
* Allow expressions in include tags
* Expose more lidar params in its XML
* Contributors: Jose Luis Blanco-Claraco

0.4.0 (2022-09-27)
------------------
* Major new release with tons of new features.
* New sensors: RGB, depth, RGB+D cameras
* Support for ROS1 and ROS2.
* Sensors now can have 3D models.
* New GUI controls to customize visualization.
* New 3 and 4 wheels differential kinematic models.
* Allow "include"s in XML files.
* Add 3D Jackal robot model.
* ROS nodes: publishers in parallel thread
* mvsim-cli new flag to enable full profiling
* Use new nanogui feature to limit GUI refresh rate
* Fix running faster than real-time
* More consistent timestamping of simulated sensors
* mvsim now shows program version info
* get_pose() service now also gets twist
* Fix elevation maps
* pybind11 per-version directory
* Remove trailing '/' in tf frame names for consistency with modern conventions.
* Rename COPYING -> LICENSE
* Contributors: Jose Luis Blanco-Claraco

0.3.2 (2022-06-21)
------------------
* Install models/ subdirectory too
* Changes towards building for both ros1 & ros2
* Copyright date bump
* Fix build and dependencies for ROS1.
* Fix build w/o python
* Fix consistent include path for installed targets
* BUGFIX: Fix random SIGSEGV due to unsafe shared global object for random number generation
* Fix no installation of mvsim_msgs python module
* Fix demo robot starts out of the map
* Contributors: Jose Luis Blanco-Claraco

0.3.1 (2022-04-25)
------------------
* update 2 robots demo
* Add pybind11 as build dep
* fix ros node compilation
* fix build w/o ros
* Fix compilation of the ROS1 node against the latest mvsim libraries
* Fix cmake policy error in pybind11
* Add missing ros deps
* Add missing build dep box2d-dev
* Update README.md
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

0.3.0 (2022-03-04)
------------------
* RGBD camera simulation
* MRPT 2.x is now required to build mvsim.
* Update build dep to mrpt2
* License changed to 3-clause BSD.
* Merge pull request `#11 <https://github.com/ual-arm-ros-pkg/mvsim/issues/11>`_ from SRai22/patch-1
  Update install.rst: needs libprotobuf-dev and libpython3-dev for building from source
* New checkboxes to see sensor poses and FOVs
* Lidar: ignore parent body option
* Lidar: realistic 3D raytrace mode
* enable textures in planes
* add support for ground and ceiling planes
* clean elevation mesh code
* save_to_rawlog option
* register callbacks instead of virtual functions
* New command "topic echo NAME"
* Add support for intangible blocks; publish relative poses
* Add support and example for standalone sensors
* allow changing the server IP or address
* add optional profiler to Client
* Protect main socket with mutex
* fix walls rendering; add new walls demo xml
* allow custom user 3D objects
* timelog format fix
* show class name in timelogger
* World: expose GUI object
* GUI and minor tweaks
* much faster models loading
* fix wrong collision resetting
* safer report collisions
* Fix usage of the update_fps parameter
* Fix build against mrpt 2.1.8
* force build against python3
* more standard python3 deb pkg generation
* solved python pkg problem in bionic
* fix python in bionic
* debian: fix python3 install dir
* add missing python3 dep
* fix deb python packaging
* remove useless cmake include
* first fully-working set_pose from python
* Progress with python wrappers
* Enhance python wrapper
* Document a minimum size limitation in box2d.
* small preliminary test for camera sensor
* Use newer mrpt-gui window manager
* Fix wallHeight wall parameter correct usage
* editor: basic rotate and move objects
* refactor gui code into smaller methods
* fix rendering of non-custom objects
* functional replace by coordinates
* Progress with replace GUI
* progress with bbox rendering
* progress with mouse move UI
* refactor: unify all simulable objects in one list
* update asserts to latest mrpt2 names
* Better service response
* safer multithread gui
* avoid possible exception in serialization
* Return collision state
* Detect and report collisions
* add setStatic method
* progress debugging ramps
* Add incremental set_pose srv
* add get_pose() service
* Import walls working
* walls progress
* progress loading wall models
* Start doxygen integration in docs
* fix not seeing the robot owns body
* subscription works; example updated
* Feature: XML variables parsing
* update pybind11
* done topic subscriptions; fix proper thread joinable checks.
* basic subscription works
* progress subscribe topics
* fix crash upon exit due to unjoined threads
* use -dbg postfix for debug libraries
* implemented command topic list
* docs on world xml parameters
* fix visualization of sensors in custom viz models
* add missing file
* Add ZMQ monitor to connections
* fix install include dir
* fix copy pb hdr files
* clear leftover traces
* auto bbox from visuals
* More modular debian packaging
* services and set_pose() is working
* progress implementing services
* Blocks and vehicles publishes their pose
* Large code refactor:
  - Use mrpt::math types for twist and points
  - Use smart pointers
  - Remove duplicated code via new methods in base class Simulable
* done with publishTopic()
* advertise topics
* done list nodes command
* implement query node list
* refactor Client without parallel thread
* refactor mvsim-cli sources
* progress server
* fix cmake exported targets
* fix install
* unregister nodes
* basic python bindings
* refactor into modules
* progress with server parsing messages
* verbosity levels in client
* ignore files
* refactor into one main cli tool: mvsim
* zmq forwards header
* add thread names
* fix build against zmq<4.4
* First zmq message interchanges
* basic server thread infraestructure
* progress defining client/server protocol
* Add alternative 3D visualization to blocks and vehicles
* start refactor for visual objects
* fix -Werror error in u18.04
* Add zmq and protobuf
* prefer std::move
* Enforce override keyword
* use system logger instead of cout
* avoid raw pointer for box2d instance
* refactor param structures
* fix opengl memory leak
* Port to nanogui
* done port to mrpt2
* prefer nullptr
* narrower mrpt dependencies
* port docs to sphinx
* add circle-ci
* reorganize badges
* fix main doc file
* fix debian installed files path
* use system libbox2d
* add debian packaging files
* modernize: cmake exported targets
* show box2d system library version
* cmake commands to lower case
* Contributors: Jose Luis Blanco-Claraco, Shravan S Rai

0.2.1 (2019-04-12)
------------------
* Fix build against latest mrpt-master & 1.5.x versions.
* add ROS build farm badges
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Julian Lopez Velasquez

0.2.0 (2018-06-28)
------------------
* fix build against mrpt1
* update to package XML format 2
* fix build in mrpt 2.0
* use docker in travis
* Allow mvsim to be built w/o ROS again
* Merge pull request `#10 <https://github.com/ual-arm-ros-pkg/mvsim/issues/10>`_ from spsancti/master
  GSoC contribution to mvsim
  See discussion thread: https://github.com/MRPT/GSoC2017-discussions/issues/2
* Added description of world files
* Added description of loggers and Ward-Iagnemma friction model
* Added refernce to Torsen-defferntial
* Added desctiption of Ackermann-drivetrain dynamics
* Added Doxyfile
* Added user manual with basic friction model described
* Added text logger for CSV format
* Add mvsim slam demo.
* fix catkin deps: it now requires mrpt_bridge
* LaserScanner: new option to make all fixtures invisible
* Contributors: Borys Tymchenko, Jose Luis Blanco Claraco, Logrus

0.1.2 (2015-05-24)
------------------
* Cleaner build against mrpt 1.3.0
* Fix build against mrpt 1.3.0
* Contributors: Jose Luis Blanco

0.1.1 (2014-12-28)
------------------
* First public release.
* Contributors: Jose Luis Blanco
