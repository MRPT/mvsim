^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mvsim
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
