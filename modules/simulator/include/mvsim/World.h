/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_body.h>
#include <box2d/b2_distance_joint.h>
#include <box2d/b2_revolute_joint.h>
#include <box2d/b2_world.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/topography/data_types.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>
#include <mvsim/Block.h>
#include <mvsim/GUIPanel.h>
#include <mvsim/HumanActor.h>
#include <mvsim/Joystick.h>
#include <mvsim/RemoteResourcesManager.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/WorldElements/WorldElementBase.h>

//
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x020f07
#include <mrpt/io/CCompressedOutputStream.h>
#else
#include <mrpt/io/CFileGZOutputStream.h>
#endif

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/Comms/Client.h>
#endif

#include <any>
#include <atomic>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

// forward declarations:
struct GLFWwindow;
namespace mrpt::imgui
{
class CImGuiSceneView;
}

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
namespace mvsim_msgs
{
class SrvGetPose;
class SrvGetPoseAnswer;
class SrvSetPose;
class SrvSetPoseAnswer;
class SrvSetControllerTwist;
class SrvSetControllerTwistAnswer;
class SrvShutdown;
class SrvShutdownAnswer;
class SrvSetLightState;
class SrvSetLightStateAnswer;
class SrvGetLightState;
class SrvGetLightStateAnswer;
}  // namespace mvsim_msgs
#endif

namespace mvsim
{
/** \defgroup mvsim_simulator_module mvsim-simulator
 *   The main module: vehicles, sensors, world objects, etc.
 */

/** \mainpage MVSim
 * This is the Doxygen-based C++ API documentation.
 * Use the links above to browse existing classes.
 *
 * Main documentation and tutorials live here: https://mvsimulator.readthedocs.io/
 */

/** Describes an inter-body joint parsed from the world XML.
 *  Supports distance (rope) and revolute (pin/hinge) joint types.
 *
 *  \ingroup mvsim_simulator_module
 */
struct WorldJoint
{
	enum class Type : uint8_t
	{
		Distance = 0,  ///< b2DistanceJoint (rope-like max length constraint)
		Revolute  ///< b2RevoluteJoint (pin/hinge)
	};

	Type type = Type::Distance;

	/// Names of the two connected Simulable objects (vehicles, blocks, etc.)
	std::string bodyA_name;
	std::string bodyB_name;

	/// Local anchor points on each body (in body-local coordinates)
	mrpt::math::TPoint2D anchorA{0, 0};
	mrpt::math::TPoint2D anchorB{0, 0};

	/// Distance joint parameters (only used when type == Distance):
	float maxLength = 5.0f;
	float minLength = 0.0f;
	float stiffness = 0.0f;
	float damping = 0.0f;

	/// Revolute joint parameters:
	bool enableLimit = false;
	float lowerAngle_deg = 0.0f;  ///< Parsed in degrees, converted to radians
	float upperAngle_deg = 0.0f;

	/// Runtime Box2D joint (owned by the b2World, do NOT delete manually)
	b2Joint* b2joint = nullptr;
};

/** Simulation happens inside a World object.
 * This is the central class for usage from user code, running the simulation,
 * loading XML models, managing GUI visualization, etc.
 * The ROS node acts as a bridge between this class and the ROS subsystem.
 *
 * See: https://mvsimulator.readthedocs.io/en/latest/world.html
 *
 * \ingroup mvsim_simulator_module
 */
class World : public mrpt::system::COutputLogger
{
   public:
	/** \name Initialization, simulation set-up
	  @{*/
	World();  //!< Default ctor: inits an empty world
	~World();  //!< Dtor.

	// Rule of Five: explicitly delete copy operations, allow move operations
	World(const World&) = delete;
	World& operator=(const World&) = delete;
	World(World&&) = delete;
	World& operator=(World&&) = delete;

	/** Resets the entire simulation environment to an empty world.
	 */
	void clear_all();

	/** Load an entire world description into this object from a specification
	 * in XML format.
	 * \param[in] xmlFileNamePath The relative or full path to the XML file.
	 * \exception std::exception On any error, with what() giving a descriptive
	 * error message
	 */
	void load_from_XML_file(const std::string& xmlFileNamePath);

	void internal_initialize();

	/** Load an entire world description into this object from a specification
	 * in XML format.
	 * \param[in] fileNameForPath Optionally, provide the full path to an XML
	 * file from which to take relative paths.
	 * \exception std::exception On any error, with what() giving a descriptive
	 * error message
	 */
	void load_from_XML(
		const std::string& xml_text, const std::string& fileNameForPath = std::string("."));
	/** @} */

	/** \name Simulation execution
	  @{*/

	/** Seconds since start of simulation. \sa get_simul_timestamp() */
	double get_simul_time() const
	{
		auto lck = mrpt::lockHelper(simul_time_mtx_);
		return simulTime_;
	}

	/// Normally should not be called by users, for internal use only.
	void force_set_simul_time(double newSimulatedTime)
	{
		auto lck = mrpt::lockHelper(simul_time_mtx_);
		simulTime_ = newSimulatedTime;
	}

	/** Get the current simulation full timestamp, computed as the
	 *  real wall clock timestamp at the beginning of the simulation,
	 *  plus the number of seconds simulation has run.
	 *  \sa get_simul_time()
	 */
	mrpt::Clock::time_point get_simul_timestamp() const
	{
		auto lck = mrpt::lockHelper(simul_time_mtx_);
		ASSERT_(simul_start_wallclock_time_.has_value());
		return mrpt::Clock::fromDouble(simulTime_ + simul_start_wallclock_time_.value());
	}

	/** Returns true once the simulation has an established wall-clock time
	 *  origin (i.e. run_simulation() has been called at least once), so that
	 *  get_simul_timestamp() can be safely queried. */
	bool has_simul_timestamp() const
	{
		auto lck = mrpt::lockHelper(simul_time_mtx_);
		return simul_start_wallclock_time_.has_value();
	}

	/** Achieved real-time factor: smoothed ratio of simulated time advanced to
	 *  wall-clock time elapsed. 1.0 means real time; below 1.0 means the
	 *  simulation is running slower than real time. \sa run_simulation() */
	double get_realtime_factor_achieved() const { return achievedRealtimeFactor_.load(); }

	/** Smoothed fraction of the wall-clock time spent inside run_simulation().
	 * Close to 1.0 means the simulation can not keep up with the requested
	 * speed, e.g. because it waits for OpenGL sensors. */
	double simulation_busy_fraction() const;

	/** Simulation time of the next sensor reading that needs OpenGL
	 * rendering (cameras, 3D lidars...), or nullopt if there are none. */
	std::optional<double> next_opengl_sensor_time() const;

	/// Simulation fixed-time interval for numerical integration
	double get_simul_timestep() const;

	/// Simulation fixed-time interval for numerical integration
	/// `0` means auto-determine as the minimum of 50 ms and the shortest sensor
	/// sample period.
	void set_simul_timestep(double timestep) { simulTimestep_ = timestep; }

	/// Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for
	/// friction, etc.
	double get_gravity() const { return gravity_; }

	/// Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for
	/// friction, etc.
	void set_gravity(double accel) { gravity_ = accel; }

	/** Runs the simulation for a given time interval (in seconds)
	 * \note The minimum simulation time is the timestep set (e.g. via
	 * set_simul_timestep()), even if time advanced further than the provided
	 * "dt".
	 */
	void run_simulation(double dt);

	void insert_vehicle(const VehicleBase::Ptr& veh);

	/** For usage in TUpdateGUIParams and \a update_GUI() */
	struct GUIKeyEvent
	{
		/// Same value as GLFW_KEY_ESCAPE
		static constexpr int KEY_ESCAPE = 256;

		/// 0=no Key. Otherwise, the GLFW key code, which is the ASCII code
		/// for (uppercase) letters and digits.
		int keycode = 0;
		bool modifierShift = false;
		bool modifierCtrl = false;
		bool modifierAlt = false;
		bool modifierSuper = false;

		GUIKeyEvent() = default;
	};

	struct TUpdateGUIParams
	{
		GUIKeyEvent keyevent;  //!< Keystrokes in the window are returned here.
		std::string msg_lines;	//!< Messages to show

		TUpdateGUIParams() = default;
	};

	/** Updates (or sets-up upon first call) the GUI visualization of the scene.
	 * \param[inout] params Optional inputs/outputs to the GUI update process.
	 * See struct for details.
	 * \note This method is prepared to be called concurrently with the
	 * simulation, and doing so is recommended to assure a smooth
	 * multi-threading simulation.
	 */
	void update_GUI(TUpdateGUIParams* params = nullptr);

	/** Opens the GUI window right away, with a "loading" message until the
	 * world is loaded by load_from_XML() and its first frame is rendered.
	 * Otherwise, the window opens in the first call to update_GUI().
	 * Call it before load_from_XML(). It does nothing in headless mode.
	 */
	void open_GUI_while_loading();

	/** Smoothed ratio of the wall-clock time spent in run_simulation() to the
	 * simulated time: above 1.0, the simulation runs slower than real time.
	 * It is measured since the world is ready (loaded, and its first frame
	 * rendered), and mostly reflects the last couple of seconds. */
	double cpu_usage() const { return cpuUsage_.load(); }

	/// The GUI window, or nullptr if it is not open.
	GLFWwindow* gui_window() const { return gui_.window; }

	const mrpt::math::TPoint3D& gui_mouse_point() const { return gui_.clickedPt; }

	/** Adds a custom dockable panel to the GUI. It can be called from any
	 * thread, before or after the GUI window opens. Its callbacks run in the
	 * GUI thread. Panels are listed in the "Window" menu, and docked in the
	 * right column the first time the default layout is built. */
	void add_gui_panel(const gui::WindowDescription& panel);

	/** Sets a function called from the GUI thread once per GUI frame, with the
	 * state of the mouse over the 3D view. Replaces any previous one. */
	void set_gui_mouse_callback(const std::function<void(const gui::MouseState&)>& callback);

	/** If !=null, a set of objects to be rendered merged with the default
	 * visualization. Lock the mutex guiUserObjectsMtx_ while writing to these
	 * objects or their children: the GUI and sensors hold it while rendering.
	 * There are two sets of objects: "viz" for visualization only, "physical"
	 * for objects which should be detected by sensors.
	 */
	mrpt::viz::CSetOfObjects::Ptr guiUserObjectsPhysical_, guiUserObjectsViz_;
	std::mutex guiUserObjectsMtx_;

	/// Update 3D vehicles, sensors, run render-based sensors, etc:
	/// Called from World_gui thread in normal mode, or mvsim-cli in headless
	/// mode.
	void internalGraphicsLoopTasksForSimulation();

	/// Frees the sensors OpenGL resources. Must be called from the same thread
	/// that called internalGraphicsLoopTasksForSimulation(), before exiting it.
	void internalFreeOpenGLResourcesForSimulation();

	void internalRunSensorsOn3DScene(mrpt::viz::Scene& physicalObjects);

	void internalUpdate3DSceneObjects(mrpt::viz::Scene& viz, mrpt::viz::Scene& physical);
	void internal_GUI_thread();
	void internal_process_pending_gui_user_tasks();

	std::mutex pendingRunSensorsOn3DSceneMtx_;
	bool pendingRunSensorsOn3DScene_ = false;

	void mark_as_pending_running_sensors_on_3D_scene()
	{
		{
			std::lock_guard<std::mutex> lck(pendingRunSensorsOn3DSceneMtx_);
			pendingRunSensorsOn3DScene_ = true;
		}
		internal_wake_up_gui_thread();
	}

	/// Makes the GUI thread attend pending tasks right away, instead of
	/// waiting for its next frame.
	void internal_wake_up_gui_thread();

	/// Called from the GUI window input callbacks:
	void internal_on_gui_key(int key, int action, int mods);
	void internal_on_gui_input_event();
	void internal_on_gui_focus(bool focused);

	/// Sets the GUI camera from the world file <gui> options.
	void internal_apply_initial_camera();
	void clear_pending_running_sensors_on_3D_scene()
	{
		std::lock_guard<std::mutex> lck(pendingRunSensorsOn3DSceneMtx_);
		pendingRunSensorsOn3DScene_ = false;
	}
	bool pending_running_sensors_on_3D_scene()
	{
		std::lock_guard<std::mutex> lck(pendingRunSensorsOn3DSceneMtx_);
		return pendingRunSensorsOn3DScene_;
	}

	std::string guiMsgLines_;
	std::mutex guiMsgLinesMtx_;

	std::thread gui_thread_;

	std::atomic_bool gui_thread_running_ = false;
	std::atomic_bool simulator_must_close_ = false;
	mutable std::mutex gui_thread_start_mtx_;

	bool simulator_must_close() const
	{
		std::lock_guard<std::mutex> lck(gui_thread_start_mtx_);
		return simulator_must_close_;
	}
	void simulator_must_close(bool value)
	{
		std::lock_guard<std::mutex> lck(gui_thread_start_mtx_);
		simulator_must_close_ = value;
	}

	void enqueue_task_to_run_in_gui_thread(const std::function<void(void)>& f) const
	{
		std::lock_guard<std::mutex> lck(guiUserPendingTasksMtx_);
		guiUserPendingTasks_.emplace_back(f);
	}

	mutable std::vector<std::function<void(void)>> guiUserPendingTasks_;
	mutable std::mutex guiUserPendingTasksMtx_;

	GUIKeyEvent lastKeyEvent_;
	std::atomic_bool lastKeyEventValid_ = false;
	std::mutex lastKeyEventMtx_;

	bool is_GUI_open() const;  //!< Return true if the GUI window is open, after
							   //! a previous call to update_GUI()

	void close_GUI();  //!< Forces closing the GUI window, if any.

	/** @} */

	/** \name Public types
	  @{*/

	/** Map 'vehicle-name' => vehicle object. See getListOfVehicles() */
	using VehicleList = std::multimap<std::string, VehicleBase::Ptr>;

	/** See getListOfWorldElements() */
	using WorldElementList = std::list<WorldElementBase::Ptr>;

	/** Map 'block-name' => block object. See getListOfBlocks()*/
	using BlockList = std::multimap<std::string, Block::Ptr>;

	/** For convenience, all elements (vehicles, world elements, blocks) are
	 * also stored here for each look-up by name */
	using SimulableList = std::multimap<std::string, Simulable::Ptr>;

	/** Map 'actor-name' => actor object. See getListOfActors() */
	using ActorList = std::multimap<std::string, HumanActor::Ptr>;

	/** @} */

	/** \name Access inner working objects
	  @{*/
	std::unique_ptr<b2World>& getBox2DWorld() { return box2d_world_; }
	const std::unique_ptr<b2World>& getBox2DWorld() const { return box2d_world_; }
	b2Body* getBox2DGroundBody() { return b2_ground_body_; }
	const VehicleList& getListOfVehicles() const { return vehicles_; }
	VehicleList& getListOfVehicles() { return vehicles_; }
	const BlockList& getListOfBlocks() const { return blocks_; }
	BlockList& getListOfBlocks() { return blocks_; }
	const WorldElementList& getListOfWorldElements() const { return worldElements_; }

	const std::vector<WorldJoint>& getListOfJoints() const { return joints_; }

	const ActorList& getListOfActors() const { return actors_; }
	ActorList& getListOfActors() { return actors_; }

	/// Always lock/unlock getListOfSimulableObjectsMtx() before using this:
	SimulableList& getListOfSimulableObjects() { return simulableObjects_; }
	const SimulableList& getListOfSimulableObjects() const { return simulableObjects_; }
	auto& getListOfSimulableObjectsMtx() { return simulableObjectsMtx_; }

	mrpt::system::CTimeLogger& getTimeLogger() { return timlogger_; }

	/** Switches a light group (`<light_group>` XML tags) of a vehicle, block,
	 * etc. on or off. It can be called from any thread.
	 * \return false if there is no such object or light group.
	 * \sa CVisualObject::setLightGroupState() */
	bool setLightGroupState(const std::string& objectName, const std::string& groupName, bool on);

	/** Whether a light group of an object is on, or empty if there is no such
	 * object or light group. */
	std::optional<bool> lightGroupState(
		const std::string& objectName, const std::string& groupName) const;

	/** Replace macros, prefix the base_path if input filename is relative, etc.
	 *  \sa xmlPathToActualPath
	 */
	std::string local_to_abs_path(const std::string& in_path) const;

	/** Parses URIs in all the forms explained in
	 * RemoteResourcesManager::resolve_path(), then passes it through
	 * local_to_abs_path().
	 *
	 *  \sa local_to_abs_path
	 */
	std::string xmlPathToActualPath(const std::string& modelURI) const;

	/** @} */

	/** \name Visitors API
	  @{*/

	using vehicle_visitor_t = std::function<void(VehicleBase&)>;
	using world_element_visitor_t = std::function<void(WorldElementBase&)>;
	using block_visitor_t = std::function<void(Block&)>;

	/** Run the user-provided visitor on each vehicle */
	void runVisitorOnVehicles(const vehicle_visitor_t& v);

	/** Run the user-provided visitor on each world element */
	void runVisitorOnWorldElements(const world_element_visitor_t& v);

	/** Run the user-provided visitor on each world block */
	void runVisitorOnBlocks(const block_visitor_t& v);

	/** @} */

	/** \name Optional user hooks
	  @{*/

	using on_observation_callback_t =
		std::function<void(const Simulable& /*veh*/, const mrpt::obs::CObservation::Ptr& /*obs*/)>;

	void registerCallbackOnObservation(const on_observation_callback_t& f)
	{
		callbacksOnObservation_.emplace_back(f);
	}

	/** Performance statistics, measured over consecutive windows of
	 * simulated time. \sa getPerformanceStats() */
	struct PerformanceStats
	{
		double window_simul_time = 0;  //!< [s] Simulated time of the window
		double window_wall_time = 0;  //!< [s] Wall-clock time of the window
		double realtime_factor = 0;	 //!< window_simul_time/window_wall_time
		size_t steps = 0;  //!< Number of physics steps
		double physics_time = 0;  //!< [s] Wall-clock time in physics steps
		double sensors_wait_time = 0;  //!< [s] Waiting for OpenGL sensors

		struct Sensor
		{
			double processing_time = 0;	 //!< [s] Wall-clock time
			size_t observations = 0;  //!< Number of generated observations
		};
		/** Per sensor, by "<vehicle>/<sensor>" */
		std::map<std::string, Sensor> sensors;
	};

	/** Statistics of the last completed window (a few seconds of simulated
	 * time). Thread-safe. */
	PerformanceStats getPerformanceStats() const;

	/** Internal: accounts the wall-clock processing time of a sensor */
	void internalAddSensorProcessingTime(const std::string& key, double seconds);

	/** Calls all registered callbacks: */
	void dispatchOnObservation(const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);

	/** @} */

	/** Connect to server, advertise topics and services, etc. per the world
	 * description loaded from XML file. */
	void connectToServer();

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mvsim::Client& commsClient() { return client_; }
	const mvsim::Client& commsClient() const { return client_; }
#endif

	void free_opengl_resources();

	auto& physical_objects_mtx() { return worldPhysicalMtx_; }

	bool headless() const { return guiOptions_.headless; }
	void headless(bool setHeadless) { guiOptions_.headless = setHeadless; }

	bool sensor_has_to_create_egl_context();

	/** Number of shadow map cascades used while rendering camera sensors. */
	int sensor_shadow_cascades() const { return lightOptions_.sensor_shadow_cascades; }

	const std::map<std::string, std::string>& user_defined_variables() const
	{
		return userDefinedVariables_;
	}

	/** If joystick usage is enabled (via XML file option, for example),
	 *  this will read the joystick state and return it. Otherwise (or on device
	 * error or disconnection), a null optional variable is returned.
	 */
	std::optional<mvsim::TJoyStickEvent> getJoystickState() const;

	bool evaluate_tag_if(const rapidxml::xml_node<char>& node) const;

	float collisionThreshold() const { return collisionThreshold_; }

	/** Returns the list of "z" coordinate or "elevations" for all simulable objects at a given
	 *  world-frame 2D coordinates (x,y). If no object reports any height, the value "0.0" will be
	 * always reported by default. In multistorey worlds, for example, this will return the height
	 * of each floor for the queried point.
	 */
	std::set<float> getElevationsAt(const mrpt::math::TPoint2D& worldXY) const;

	/// with query points the center of a wheel, this returns the highest "ground" under it, or .0
	/// if nothing found.
	float getHighestElevationUnder(const mrpt::math::TPoint3Df& queryPt) const;

	/** Must be called when world elements are added or moved, so the spatial
	 * index used by elevation queries is rebuilt. */
	void invalidateElevationIndex() { elevationIndexIsUpToDate_ = false; }

	void internal_simul_pre_step_terrain_elevation();

	/** Query all mvsim::WorldElementBase objects for a given custom property at the specific 3D
	 * location. It returns nullopt if no object defines this property.
	 * \sa WorldElementBase::queryProperty()
	 */
	std::optional<std::any> getPropertyAt(
		const std::string& propertyName, const mrpt::math::TPoint3D& worldXYZ) const;

   private:
	friend class VehicleBase;
	friend class Block;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mvsim::Client client_{"World"};
#endif

	std::vector<on_observation_callback_t> callbacksOnObservation_;

	// -------- World Params ----------
	/** Gravity acceleration (Default=9.81 m/s^2). Used to evaluate weights for
	 * friction, etc. */
	double gravity_ = 9.81;

	/** Simulation fixed-time interval for numerical integration.
	 * `0` means auto-determine as the minimum of 50 ms and the shortest sensor
	 * sample period.
	 */
	mutable double simulTimestep_ = 0;

	mutable bool joystickEnabled_ = false;
	mutable std::optional<Joystick> joystick_;

	/** Velocity and position iteration count (refer to libbox2d docs) */
	int b2dVelIters_ = 8, b2dPosIters_ = 3;

	/** Distance between two body edges to be considered a collision. */
	float collisionThreshold_ = 0.03f;

	std::string serverAddress_ = "localhost";

	/** If non-empty, all observations will be saved to a .rawlog */
	std::string save_to_rawlog_;

	double rawlog_odometry_rate_ = 10.0;  //!< In Hz.

	/** If non-empty, the ground truth trajectory of all vehicles will be saved
	 * to a text file in the TUM trajectory format */
	std::string save_ground_truth_trajectory_;

	double ground_truth_rate_ = 50.0;  //!< In Hz.

	double max_slope_to_collide_ = 0.30;
	double min_slope_to_collide_ = -0.50;

	const TParameterDefinitions otherWorldParams_ = {
		{"server_address", {"%s", &serverAddress_}},
		{"gravity", {"%lf", &gravity_}},
		{"simul_timestep", {"%lf", &simulTimestep_}},
		{"b2d_vel_iters", {"%i", &b2dVelIters_}},
		{"b2d_pos_iters", {"%i", &b2dPosIters_}},
		{"collision_threshold", {"%f", &collisionThreshold_}},
		{"joystick_enabled", {"%bool", &joystickEnabled_}},
		{"save_to_rawlog", {"%s", &save_to_rawlog_}},
		{"rawlog_odometry_rate", {"%lf", &rawlog_odometry_rate_}},
		{"save_ground_truth_trajectory", {"%s", &save_ground_truth_trajectory_}},
		{"ground_truth_rate", {"%lf", &ground_truth_rate_}},
		{"max_slope_to_collide", {"%lf", &max_slope_to_collide_}},
		{"min_slope_to_collide", {"%lf", &min_slope_to_collide_}},
	};

	/** User-defined variables as defined via `<variable name='' value='' />`
	 * tags in the World xml file, for use within `$f{}` expressions */
	std::map<std::string, std::string> userDefinedVariables_;

	/** In seconds, real simulation time since beginning (may be different than
	 * wall-clock time because of time warp, etc.) */
	double simulTime_ = 0;
	std::optional<double> simul_start_wallclock_time_;
	std::mutex simul_time_mtx_;

	/** Achieved real-time factor (simulated seconds advanced per wall-clock
	 * second), exponentially smoothed. 1.0 = real time; below 1.0 = running
	 * slower than real time (e.g. CPU-bound). Updated by run_simulation(). */
	std::atomic<double> achievedRealtimeFactor_{1.0};
	/// See simulation_busy_fraction()
	std::atomic<double> simulBusyFraction_{0.0};
	/// Wall-clock start of the run_simulation() call in progress, or 0.
	std::atomic<double> runSimulStartWallclock_{0.0};
	static constexpr double BUSY_FRACTION_TIME_CONSTANT = 1.0;	// [s]
	std::optional<double> lastRunSimulWallclock_;

	/// Set by open_GUI_while_loading(), until load_from_XML() ends.
	std::atomic_bool guiWaitsForWorldLoad_ = false;

	/// Wall-clock time when the world got ready: loaded, and its first frame
	/// rendered (or, in headless mode, its sensors first updated). 0=not yet.
	std::atomic<double> worldReadyWallclock_{0};

	/// See cpu_usage(). Updated by run_simulation():
	std::atomic<double> cpuUsage_{0};
	double cpuUsageSumCpuTime_ = 0;
	double cpuUsageSumSimulTime_ = 0;
	bool highCpuUsageChecked_ = false;

	void updateCpuUsage(double simulTime, double cpuTime);

	/** Path from which to take relative directories. */
	std::string basePath_{"."};

	/// This private container will be filled with objects in the public
	/// gui_user_objects_
	mrpt::viz::CSetOfObjects::Ptr glUserObjsPhysical_ = mrpt::viz::CSetOfObjects::Create();
	mrpt::viz::CSetOfObjects::Ptr glUserObjsViz_ = mrpt::viz::CSetOfObjects::Create();

	// ------- GUI options -----
	struct TGUI_Options
	{
		unsigned int win_w = 800, win_h = 600;
		bool start_maximized = true;
		int refresh_fps = 20;
		bool ortho = false;
		bool show_forces = false;
		bool show_sensor_points = true;
		bool show_sensor_previews = true;
		bool show_gui_panels = true;
		bool show_trajectories = false;
		double force_scale = 0.01;	//!< In meters/Newton
		double camera_distance = 80.0;
		double camera_azimuth_deg = 45.0;
		double camera_elevation_deg = 40.0;
		double fov_deg = 60.0;
		float clip_plane_min = 0.05f;
		float clip_plane_max = 10e3f;
		mrpt::math::TPoint3D camera_point_to{0, 0, 0};
		std::string follow_vehicle;	 //!< Vehicle name to follow (empty=none)
		bool headless = false;

		const TParameterDefinitions params = {
			{"win_w", {"%u", &win_w}},
			{"win_h", {"%u", &win_h}},
			{"ortho", {"%bool", &ortho}},
			{"show_forces", {"%bool", &show_forces}},
			{"show_sensor_points", {"%bool", &show_sensor_points}},
			{"show_sensor_previews", {"%bool", &show_sensor_previews}},
			{"show_gui_panels", {"%bool", &show_gui_panels}},
			{"show_trajectories", {"%bool", &show_trajectories}},
			{"force_scale", {"%lf", &force_scale}},
			{"fov_deg", {"%lf", &fov_deg}},
			{"follow_vehicle", {"%s", &follow_vehicle}},
			{"start_maximized", {"%bool", &start_maximized}},
			{"refresh_fps", {"%i", &refresh_fps}},
			{"headless", {"%bool", &headless}},
			{"clip_plane_min", {"%f", &clip_plane_min}},
			{"clip_plane_max", {"%f", &clip_plane_max}},
			{"cam_distance", {"%lf", &camera_distance}},
			{"cam_azimuth", {"%lf", &camera_azimuth_deg}},
			{"cam_elevation", {"%lf", &camera_elevation_deg}},
			{"cam_point_to", {"%point3d", &camera_point_to}},
		};

		TGUI_Options() = default;
		void parse_from(const rapidxml::xml_node<char>& node, COutputLogger& logger);
	};

	/** Some of these options are only used the first time the GUI window is
	 * created. */
	TGUI_Options guiOptions_;

	struct LightOptions
	{
		LightOptions() = default;

		void parse_from(const rapidxml::xml_node<char>& node, COutputLogger& logger);

		bool enable_shadows = true;
		int shadow_map_size = 2048;

		/// Turn off the point and spot lights if the simulation is slower
		/// than real time during its first seconds:
		bool disable_lights_on_high_cpu_usage = true;

		/// Cascaded shadow map splits (1-4) for the GUI view.
		int shadow_cascades = 4;

		/// Cascaded shadow map splits (1-4) for camera sensors. Each one
		/// costs a full shadow map pass per rendered image.
		int sensor_shadow_cascades = 1;

		double light_azimuth = mrpt::DEG2RAD(45.0);
		double light_elevation = mrpt::DEG2RAD(70.0);

		float light_clip_plane_min = 0.1f;
		float light_clip_plane_max = 900.0f;

		float shadow_bias = 1e-5;
		float shadow_bias_cam2frag = 1e-5;
		float shadow_bias_normal = 1e-4;

		mrpt::img::TColor light_color = {0xff, 0xff, 0xff, 0xff};
		float light_ambient = 0.4f;
		float light_diffuse = 0.8f;
		float light_specular = 0.6f;

		/// Hemisphere ambient sky color (surfaces facing up)
		mrpt::img::TColor ambient_sky_color = {0xe0, 0xe8, 0xff, 0xff};
		/// Hemisphere ambient ground color (surfaces facing down)
		mrpt::img::TColor ambient_ground_color = {0x40, 0x3a, 0x30, 0xff};

		float eye_distance_to_shadow_map_extension = 2.0f;	//!< [m/m]
		float minimum_shadow_map_extension_ratio = 0.005f;	//!< [0,1]

		/** Additional light sources (point and spot) parsed from XML */
		std::vector<mrpt::viz::TLight> extra_lights;

		const TParameterDefinitions params = {
			{"enable_shadows", {"%bool", &enable_shadows}},
			{"disable_lights_on_high_cpu_usage", {"%bool", &disable_lights_on_high_cpu_usage}},
			{"shadow_map_size", {"%i", &shadow_map_size}},
			{"shadow_cascades", {"%i", &shadow_cascades}},
			{"sensor_shadow_cascades", {"%i", &sensor_shadow_cascades}},
			{"light_azimuth_deg", {"%lf_deg", &light_azimuth}},
			{"light_elevation_deg", {"%lf_deg", &light_elevation}},
			{"light_clip_plane_min", {"%f", &light_clip_plane_min}},
			{"light_clip_plane_max", {"%f", &light_clip_plane_max}},
			{"light_color", {"%color", &light_color}},
			{"light_diffuse", {"%f", &light_diffuse}},
			{"light_specular", {"%f", &light_specular}},
			{"shadow_bias", {"%f", &shadow_bias}},
			{"shadow_bias_cam2frag", {"%f", &shadow_bias_cam2frag}},
			{"shadow_bias_normal", {"%f", &shadow_bias_normal}},
			{"light_ambient", {"%f", &light_ambient}},
			{"ambient_sky_color", {"%color", &ambient_sky_color}},
			{"ambient_ground_color", {"%color", &ambient_ground_color}},
			{"eye_distance_to_shadow_map_extension", {"%f", &eye_distance_to_shadow_map_extension}},
			{"minimum_shadow_map_extension_ratio", {"%f", &minimum_shadow_map_extension_ratio}},
		};
	};

	/** Options for lights */
	LightOptions lightOptions_;

   public:
	// Options for simulating GNSS (GPS) sensors.
	struct GeoreferenceOptions
	{
		GeoreferenceOptions() = default;

		void parse_from(const rapidxml::xml_node<char>& node, COutputLogger& logger);

		/// Latitude/longitude/height of the world (0,0,0) frame.
		mrpt::topography::TGeodeticCoords georefCoord;

		/** Optional world rotation (in radians, in degrees in the XML file)
		 *  wrt ENU frame: 0 (default) means +X points East.
		 */
		double world_to_enu_rotation = .0;

		/** Is world in UTM coordinates? */
		bool world_is_utm = false;

		/** The UTM coords of georefCoord (calculated on start up) */
		mrpt::topography::TUTMCoords utmRef;
		int utm_zone = 0;  // auto calculated
		char utm_band = 'X';  // auto calculated

		const TParameterDefinitions params = {
			{"latitude", {"%lf", &georefCoord.lat.decimal_value}},
			{"longitude", {"%lf", &georefCoord.lon.decimal_value}},
			{"height", {"%lf", &georefCoord.height}},
			{"world_to_enu_rotation_deg", {"%lf_deg", &world_to_enu_rotation}},
			{"world_is_utm", {"%bool", &world_is_utm}},
		};
	};

	const GeoreferenceOptions& georeferenceOptions() const { return georeferenceOptions_; }

	/// (See docs for worldRenderOffset_)
	mrpt::math::TVector3D worldRenderOffset() const
	{
		return worldRenderOffset_ ? *worldRenderOffset_ : mrpt::math::TVector3D(0, 0, 0);
	}
	mrpt::math::TPose3D applyWorldRenderOffset(mrpt::math::TPose3D p) const
	{
		const auto t = worldRenderOffset();
		p.x += t.x;
		p.y += t.y;
		p.z += t.z;
		return p;
	}
	mrpt::poses::CPose3D applyWorldRenderOffset(mrpt::poses::CPose3D p) const
	{
		const auto t = worldRenderOffset();
		p.x_incr(t.x);
		p.y_incr(t.y);
		p.z_incr(t.z);
		return p;
	}
	/// (See docs for worldRenderOffset_)
	void worldRenderOffsetPropose(const mrpt::math::TVector3D& v)
	{
		if (!worldRenderOffset_)
		{
			worldRenderOffset_ = v;
		}
	}

   private:
	/** Options for lights */
	GeoreferenceOptions georeferenceOptions_;

	// -------- World contents ----------
	/** Mutex protecting simulation objects from multi-thread access */
	std::recursive_mutex world_cs_;

	/** Box2D dynamic simulator instance */
	std::unique_ptr<b2World> box2d_world_;

	/** Used to declare friction between vehicles-ground*/
	b2Body* b2_ground_body_ = nullptr;

	VehicleList vehicles_;
	WorldElementList worldElements_;
	BlockList blocks_;
	ActorList actors_;

	/// Inter-body joints (distance / revolute)
	std::vector<WorldJoint> joints_;

	bool initialized_ = false;

	// List of all objects above (vehicles, world_elements, blocks), but as
	// shared_ptr to their Simulable interfaces, so we can easily iterate on
	// this list only for common tasks:
	SimulableList simulableObjects_;
	mutable std::mutex simulableObjectsMtx_;

	/** Runs one individual time step */
	void internal_one_timestep(double dt);

	std::mutex simulationStepRunningMtx_;

	// A 2D-hash table of objects
	struct lut_2d_coordinates_t
	{
		int32_t x, y;

		bool operator==(const lut_2d_coordinates_t& o) const noexcept
		{
			return (x == o.x && y == o.y);
		}
	};

	static lut_2d_coordinates_t xy_to_lut_coords(const mrpt::math::TPoint2Df& p);

	struct LutIndexHash
	{
		std::size_t operator()(const lut_2d_coordinates_t& p) const noexcept
		{
			// These are the implicit assumptions of the reinterpret cast below:
			static_assert(sizeof(int32_t) == sizeof(uint32_t));
			static_assert(offsetof(lut_2d_coordinates_t, x) == 0 * sizeof(uint32_t));
			static_assert(offsetof(lut_2d_coordinates_t, y) == 1 * sizeof(uint32_t));

			const uint32_t* vec = reinterpret_cast<const uint32_t*>(&p);
			return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663);
		}
		/// k1 < k2? for std::map containers
		bool operator()(
			const lut_2d_coordinates_t& k1, const lut_2d_coordinates_t& k2) const noexcept
		{
			if (k1.x != k2.x)
			{
				return k1.x < k2.x;
			}
			return k1.y < k2.y;
		}
	};

	using LUTCache =
		std::unordered_map<lut_2d_coordinates_t, std::vector<Simulable::Ptr>, LutIndexHash>;

	/// Ensure the cache is built and up-to-date, then return it:
	const LUTCache& getLUTCacheOfObjects() const;

	mutable LUTCache lut2d_objects_;
	mutable bool lut2d_objects_is_up_to_date_ = false;

	/** Objects covering more cells than this are not indexed by cell, but
	 * queried everywhere, to keep the indices small. */
	static constexpr std::size_t MAX_LUT_CELLS_PER_OBJECT = 4096;

	/** Blocks too large to be indexed by cell */
	mutable std::vector<Simulable::Ptr> lut2d_oversized_objects_;

	void internal_update_lut_cache() const;

	/** Spatial index of the world elements, for elevation queries: elements
	 * by 2D cell, plus those without a known bounding box. */
	mutable std::unordered_map<lut_2d_coordinates_t, std::vector<WorldElementBase*>, LutIndexHash>
		elevationIndex_;
	mutable std::vector<WorldElementBase*> elevationIndexUnbounded_;
	mutable std::atomic_bool elevationIndexIsUpToDate_ = false;
	mutable std::shared_mutex elevationIndexMtx_;

	void internal_update_elevation_index() const;

	/** Calls f(z) for each elevation at the given point. */
	template <typename Functor>
	void forEachElevationAt(const mrpt::math::TPoint2D& worldXY, const Functor& f) const;

	/** GUI stuff  */
	struct GUI
	{
		explicit GUI(World& parent);
		~GUI();

		GLFWwindow* window = nullptr;
		/// Set while `window` exists, so other threads can wake up the GUI.
		/// Both protected by windowMtx.
		bool windowReady = false;
		std::mutex windowMtx;

		/// Set by close_GUI(): the window gets hidden, but the GUI thread
		/// keeps running the OpenGL sensors.
		std::atomic_bool hideRequested = false;
		bool hidden = false;

		/// Renders worldVisual_ behind all panels:
		std::unique_ptr<mrpt::imgui::CImGuiSceneView> sceneView;

		/// Mouse over the 3D view, and the ray (scene coordinates) under it:
		bool scene_hovered() const;
		std::optional<mrpt::math::TLine3D> scene_mouse_ray() const;
		/// For MRPT versions without CImGuiSceneView::mouseRay(): the 3D view
		/// widget, from the last frame.
		bool legacySceneHovered = false;
		float legacySceneX = 0;
		float legacySceneY = 0;

		/// Ground point under the mouse cursor:
		mrpt::math::TPoint3D clickedPt{0, 0, 0};

		/// Set from GLFW input callbacks; used to raise the frame rate while
		/// the user interacts with the window.
		std::atomic_bool gotInputEvents = false;
		bool windowFocused = false;

		/// Smoothed time a frame occupies the GUI thread and the GPU [s]
		double frameCost = 0.02;
		/// GPU time stamp queries: two alternating [begin,end] pairs
		unsigned int gpuQueries[2][2] = {{0, 0}, {0, 0}};
		bool gpuQueriesIssued[2] = {false, false};
		int gpuQueryIdx = 0;
		double lastGpuFrameTime = 0;  //!< [s]

		// Panels visibility:
		bool showWorld = true;
		bool showInspector = true;
		bool showLighting = true;
		bool showMessages = true;
		bool resetLayoutRequested = false;

		// View options (others are in guiOptions_):
		bool showSensorPoses = false;
		bool showSensorFOVs = false;
		bool showCollisionShapes = false;
		float sunIntensity = 1.0f;

		std::string worldFilter;  //!< "World" panel search box

		/// Snapshot of the world objects shown in the panels. Only refreshed
		/// while the simulation thread does not hold the list of objects, so
		/// the GUI thread (which also renders the OpenGL sensors) never waits
		/// for a simulation step.
		struct ObjectsSnapshot
		{
			using List = std::vector<std::pair<std::string, Simulable::Ptr>>;
			List vehicles;
			List blocks;
			List actors;
			List elements;

			struct LightGroup
			{
				std::string label;
				Simulable::Ptr owner;  //!< keeps `visual` alive
				CVisualObject* visual = nullptr;
				std::string group;
			};
			std::vector<LightGroup> lightGroups;
		};
		ObjectsSnapshot objects;
		void refresh_objects_snapshot();

		// Selected object in the "World" panel:
		Simulable::Ptr selected;
		std::string selectedName;
		CVisualObject* selectedVisual = nullptr;
		/// If true, the selected object follows the mouse until clicking:
		bool placingWithMouse = false;

		/// Live camera/depth images, one window per sensor:
		struct SensorPreview
		{
			std::string title;	//!< "vehicle/sensor"
			bool open = true;
			bool visible = false;  //!< Not collapsed nor hidden, last frame
			/// [0]=RGB, [1]=depth. GL textures (0=none yet).
			unsigned int tex[2] = {0, 0};
			int width[2] = {0, 0};
			int height[2] = {0, 0};
		};
		std::map<std::string, SensorPreview> sensorPreviews;

		void select(const std::string& name, const Simulable::Ptr& obj);

		void draw_frame();
		void draw_loading_frame();
		void draw_menu_bar();
		void draw_status_bar();
		void draw_dockspace_and_background();
		void draw_world_panel();
		void draw_inspector_panel();
		void draw_lighting_panel();
		void draw_messages_panel();
		void draw_sensor_previews();

		void handle_mouse_operations();

		/// Custom panels (see add_gui_panel()). Only accessed from the GUI thread.
		struct UserPanel
		{
			gui::WindowDescription desc;
			/// Unique ImGui ID: the title, plus a suffix for repeated titles.
			/// Stable across runs, so the saved layout applies.
			std::string id;
			bool open = true;
			std::map<std::string, bool> checkStates;  //!< By widget id
		};
		std::vector<UserPanel> userPanels;
		std::function<void(const gui::MouseState&)> mouseCallback;
		void draw_user_panels();

		/// False if the preview exists but is not visible, so the image
		/// does not need to be prepared.
		bool preview_needs_update(const std::string& previewName, int slot) const;
		void update_preview_texture(
			const std::string& previewName, int slot, const mrpt::img::CImage& im,
			bool startVisible);
		void free_preview_textures();

	   private:
		World& parent_;

		unsigned int dockspaceId_ = 0;
		unsigned int dockLeftTopId_ = 0;
		unsigned int dockLeftMiddleId_ = 0;
		unsigned int dockLeftBottomId_ = 0;
		unsigned int dockRightId_ = 0;

		void build_default_layout();
		void show_performance_tooltip();
		/// Docks a window in the right column, unless it has saved settings.
		void dock_new_window_right(const std::string& title);
	};
	GUI gui_{*this};  //!< gui state

	/** 3D scene with all visual objects (vehicles, obstacles, markers, etc.)
	 *  \sa worldPhysical_
	 */
	mrpt::viz::Scene::Ptr worldVisual_ = mrpt::viz::Scene::Create();

	/** 3D scene with all physically observable objects: we will use this
	 * scene as input to simulated sensors like cameras, where we don't wont
	 * to see visualization marks, etc.
	 * \sa world_visual_
	 */
	mrpt::viz::Scene worldPhysical_;
	std::recursive_mutex worldPhysicalMtx_;

	/// World coordinates offset for rendering. Useful mainly to keep numerical accuracy
	/// in the OpenGL pipeline (using "floats") when using UTM world coordinates.
	/// All coordinates to be send to OpenGL must **add** this number.
	/// It is automatically set via calling worldRenderOffsetPropose()
	/// and must be retrieved via worldRenderOffset()
	std::optional<mrpt::math::TVector3D> worldRenderOffset_;

	/// Updated in internal_one_step()
	std::map<std::string, mrpt::math::TPose3D> copy_of_objects_dynstate_pose_;
	std::map<std::string, mrpt::math::TTwist2D> copy_of_objects_dynstate_twist_;
	std::set<std::string> copy_of_objects_had_collision_;

	/// See sensor_has_to_create_egl_context()
	bool eglContextCreated_ = false;

	mutable std::mutex perfStatsMtx_;
	PerformanceStats perfStatsCurrent_, perfStatsLast_;
	std::optional<double> perfWindowStartSim_, perfWindowStartWall_;
	void internalUpdatePerformanceStats(double physicsTime, double sensorsWaitTime);
	std::recursive_mutex copy_of_objects_dynstate_mtx_;

	std::set<std::string> reset_collision_flags_;
	std::mutex reset_collision_flags_mtx_;

	void internal_gui_on_observation(const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);
	void internal_gui_on_observation_3Dscan(
		const Simulable& veh, const std::shared_ptr<mrpt::obs::CObservation3DRangeScan>& obs);
	void internal_gui_on_observation_image(
		const Simulable& veh, const std::shared_ptr<mrpt::obs::CObservationImage>& obs);

	/** Looks up, among veh's sensors, the one with the given sensorLabel
	 * (nullptr if not found). */
	static const SensorBase* internal_gui_find_sensor(
		const Simulable& veh, const std::string& sensorLabel);

	/** Changes the light source direction from azimuth and elevation angles (in
	 * radians) */
	void setLightDirectionFromAzimuthElevation(const float azimuth, const float elevation);

	/** Scales the diffuse and specular intensities of the directional light
	 * (from the world XML options) by the given factor. */
	void setLightIntensityFactor(const float factor);

	/** Changes the ambient light intensity (both viewports). */
	void setLightAmbient(const float ambient);

	/// Applies lightOptions_ to the visual and physical world viewports.
	void applyLightOptions();

	/** Turns on or off the point and spot lights from the world XML <lights>
	 * tag (both viewports). */
	void setPointAndSpotLightsEnabled(bool enabled);
	bool pointAndSpotLightsEnabled_ = true;

	/// The point and spot lights from the world XML, in rendering coordinates.
	std::vector<mrpt::viz::TLight> pointAndSpotLightsForRendering() const;

	/** @} */  // end GUI stuff

	mrpt::system::CTimeLogger timlogger_{true /*enabled*/, "mvsim::World"};
	mrpt::system::CTicTac timer_iteration_;

	void process_load_walls(const rapidxml::xml_node<char>& node);
	void insertBlock(const Block::Ptr& block);

	struct XmlParserContext
	{
		XmlParserContext(const rapidxml::xml_node<char>* n, const std::string& basePath)
			: node(n), currentBasePath(basePath)
		{
		}

		const rapidxml::xml_node<char>* node = nullptr;
		const std::string currentBasePath;
	};

	/// This will parse a main XML file, or its included
	void internal_recursive_parse_XML(const XmlParserContext& ctx);

	using xml_tag_parser_function_t = std::function<void(const XmlParserContext&)>;

	std::map<std::string, xml_tag_parser_function_t> xmlParsers_;

	void register_standard_xml_tag_parsers();

	void register_tag_parser(const std::string& xmlTagName, const xml_tag_parser_function_t& f)
	{
		xmlParsers_.emplace(xmlTagName, f);
	}
	void register_tag_parser(
		const std::string& xmlTagName, void (World::*f)(const XmlParserContext& ctx))
	{
		xmlParsers_.emplace(
			xmlTagName, [this, f](const XmlParserContext& ctx) { (this->*f)(ctx); });
	}

	// ======== XML parser tags ========
	void parse_tag_element(const XmlParserContext& ctx);  //!< `<element>`
	void parse_tag_vehicle(const XmlParserContext& ctx);  //!< `<vehicle>`
	/** <vehicle:class> */
	void parse_tag_vehicle_class(const XmlParserContext& ctx);
	void parse_tag_sensor(const XmlParserContext& ctx);	 //!<  `<sensor>`
	void parse_tag_block(const XmlParserContext& ctx);
	void parse_tag_block_class(const XmlParserContext& ctx);
	void parse_tag_gui(const XmlParserContext& ctx);
	void parse_tag_lights(const XmlParserContext& ctx);
	void parse_tag_georeference(const XmlParserContext& ctx);
	void parse_tag_walls(const XmlParserContext& ctx);
	void parse_tag_include(const XmlParserContext& ctx);
	void parse_tag_variable(const XmlParserContext& ctx);
	void parse_tag_for(const XmlParserContext& ctx);
	void parse_tag_if(const XmlParserContext& ctx);
	void parse_tag_marker(const XmlParserContext& ctx);
	void parse_tag_joint(const XmlParserContext& ctx);	//!< `<joint>`
	void parse_tag_actor(const XmlParserContext& ctx);
	void parse_tag_actor_class(const XmlParserContext& ctx);
	void parse_tag_remote_resources(const XmlParserContext& ctx);  //!< `<remote_resources>`

	// ======== end of XML parser tags ========

	mutable RemoteResourcesManager remoteResources_;

	void internalOnObservation(const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);

	void internalPostSimulStepForRawlog();
	void internalPostSimulStepForTrajectory();

	std::mutex rawlog_io_mtx_;
#if MRPT_VERSION >= 0x020f07
	std::map<std::string, std::shared_ptr<mrpt::io::CCompressedOutputStream>> rawlog_io_per_veh_;
#else
	std::map<std::string, std::shared_ptr<mrpt::io::CFileGZOutputStream>> rawlog_io_per_veh_;
#endif
	std::optional<double> rawlog_last_odom_time_;

	std::mutex gt_io_mtx_;
	std::map<std::string, std::fstream> gt_io_per_veh_;
	std::optional<double> gt_last_time_;

	// ============ Elevation Field Collision artifacts ==============
	struct TFixturePtr
	{
		TFixturePtr() = default;
		b2Fixture* fixture = nullptr;
	};
	struct TInfoPerCollidableobj
	{
		TInfoPerCollidableobj() = default;

		mrpt::poses::CPose3D pose;
		b2Body* collide_body = nullptr;
		double representativeHeight = 0.01;
		double maxWorkableStepHeight = 0.10;
		double speed = .0;
		mrpt::math::TPolygon2D contour;
		const std::vector<float>* wheel_heights = nullptr;
		std::vector<float> contour_heights;
		std::vector<TFixturePtr> collide_fixtures;
	};
	std::vector<std::optional<TInfoPerCollidableobj>> obstacles_for_each_obj_;
	// ============ end of elevation field collision =================

	// Services:
	void internal_advertiseServices();	// called from connectToServer()

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

	mvsim_msgs::SrvSetPoseAnswer srv_set_pose(const mvsim_msgs::SrvSetPose& req);
	mvsim_msgs::SrvGetPoseAnswer srv_get_pose(const mvsim_msgs::SrvGetPose& req);
	mvsim_msgs::SrvSetControllerTwistAnswer srv_set_controller_twist(
		const mvsim_msgs::SrvSetControllerTwist& req);
	mvsim_msgs::SrvShutdownAnswer srv_shutdown(const mvsim_msgs::SrvShutdown& req);
	mvsim_msgs::SrvSetLightStateAnswer srv_set_light_state(const mvsim_msgs::SrvSetLightState& req);
	mvsim_msgs::SrvGetLightStateAnswer srv_get_light_state(const mvsim_msgs::SrvGetLightState& req);
#endif
};
}  // namespace mvsim
