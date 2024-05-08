/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_body.h>
#include <box2d/b2_world.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Block.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Joystick.h>
#include <mvsim/RemoteResourcesManager.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <functional>
#include <list>
#include <map>

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
// forward declarations:
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
}  // namespace mvsim_msgs
#endif

namespace mvsim
{
/** Simulation happens inside a World object.
 * This is the central class for usage from user code, running the simulation,
 * loading XML models, managing GUI visualization, etc.
 * The ROS node acts as a bridge between this class and the ROS subsystem.
 *
 * See: https://mvsimulator.readthedocs.io/en/latest/world.html
 *
 */
class World : public mrpt::system::COutputLogger
{
   public:
	/** \name Initialization, simulation set-up
	  @{*/
	World();  //!< Default ctor: inits an empty world
	~World();  //!< Dtor.

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
		const std::string& xml_text,
		const std::string& fileNameForPath = std::string("."));
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
		return mrpt::Clock::fromDouble(
			simulTime_ + simul_start_wallclock_time_.value());
	}

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

	/** For usage in TUpdateGUIParams and \a update_GUI() */
	struct TGUIKeyEvent
	{
		int keycode = 0;  //!< 0=no Key. Otherwise, ASCII code.
		bool modifierShift = false;
		bool modifierCtrl = false;
		bool modifierAlt = false;
		bool modifierSuper = false;

		TGUIKeyEvent() = default;
	};

	struct TUpdateGUIParams
	{
		TGUIKeyEvent keyevent;	//!< Keystrokes in the window are returned here.
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

	const mrpt::gui::CDisplayWindowGUI::Ptr& gui_window() const
	{
		return gui_.gui_win;
	}

	const mrpt::math::TPoint3D& gui_mouse_point() const
	{
		return gui_.clickedPt;
	}

	/** If !=null, a set of objects to be rendered merged with the default
	 * visualization. Lock the mutex gui_user_objects_mtx_ while writing.
	 * There are two sets of objects: "viz" for visualization only, "physical"
	 * for objects which should be detected by sensors.
	 */
	mrpt::opengl::CSetOfObjects::Ptr guiUserObjectsPhysical_,
		guiUserObjectsViz_;
	std::mutex guiUserObjectsMtx_;

	/// Update 3D vehicles, sensors, run render-based sensors, etc:
	/// Called from World_gui thread in normal mode, or mvsim-cli in headless
	/// mode.
	void internalGraphicsLoopTasksForSimulation();

	void internalRunSensorsOn3DScene(
		mrpt::opengl::COpenGLScene& physicalObjects);

	void internalUpdate3DSceneObjects(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical);
	void internal_GUI_thread();
	void internal_process_pending_gui_user_tasks();

	std::mutex pendingRunSensorsOn3DSceneMtx_;
	bool pendingRunSensorsOn3DScene_ = false;

	void mark_as_pending_running_sensors_on_3D_scene()
	{
		pendingRunSensorsOn3DSceneMtx_.lock();
		pendingRunSensorsOn3DScene_ = true;
		pendingRunSensorsOn3DSceneMtx_.unlock();
	}
	void clear_pending_running_sensors_on_3D_scene()
	{
		pendingRunSensorsOn3DSceneMtx_.lock();
		pendingRunSensorsOn3DScene_ = false;
		pendingRunSensorsOn3DSceneMtx_.unlock();
	}
	bool pending_running_sensors_on_3D_scene()
	{
		pendingRunSensorsOn3DSceneMtx_.lock();
		bool ret = pendingRunSensorsOn3DScene_;
		pendingRunSensorsOn3DSceneMtx_.unlock();
		return ret;
	}

	std::string guiMsgLines_;
	std::mutex guiMsgLinesMtx_;

	std::thread gui_thread_;

	std::atomic_bool gui_thread_running_ = false;
	std::atomic_bool simulator_must_close_ = false;
	mutable std::mutex gui_thread_start_mtx_;

	bool simulator_must_close() const
	{
		gui_thread_start_mtx_.lock();
		const bool v = simulator_must_close_;
		gui_thread_start_mtx_.unlock();
		return v;
	}
	void simulator_must_close(bool value)
	{
		gui_thread_start_mtx_.lock();
		simulator_must_close_ = value;
		gui_thread_start_mtx_.unlock();
	}

	void enqueue_task_to_run_in_gui_thread(const std::function<void(void)>& f)
	{
		guiUserPendingTasksMtx_.lock();
		guiUserPendingTasks_.emplace_back(f);
		guiUserPendingTasksMtx_.unlock();
	}

	std::vector<std::function<void(void)>> guiUserPendingTasks_;
	std::mutex guiUserPendingTasksMtx_;

	TGUIKeyEvent lastKeyEvent_;
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

	/** @} */

	/** \name Access inner working objects
	  @{*/
	std::unique_ptr<b2World>& getBox2DWorld() { return box2d_world_; }
	const std::unique_ptr<b2World>& getBox2DWorld() const
	{
		return box2d_world_;
	}
	b2Body* getBox2DGroundBody() { return b2_ground_body_; }
	const VehicleList& getListOfVehicles() const { return vehicles_; }
	VehicleList& getListOfVehicles() { return vehicles_; }
	const BlockList& getListOfBlocks() const { return blocks_; }
	BlockList& getListOfBlocks() { return blocks_; }
	const WorldElementList& getListOfWorldElements() const
	{
		return worldElements_;
	}

	/// Always lock/unlock getListOfSimulableObjectsMtx() before using this:
	SimulableList& getListOfSimulableObjects() { return simulableObjects_; }
	const SimulableList& getListOfSimulableObjects() const
	{
		return simulableObjects_;
	}
	auto& getListOfSimulableObjectsMtx() { return simulableObjectsMtx_; }

	mrpt::system::CTimeLogger& getTimeLogger() { return timlogger_; }

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

	using on_observation_callback_t = std::function<void(
		const Simulable& /*veh*/, const mrpt::obs::CObservation::Ptr& /*obs*/)>;

	void registerCallbackOnObservation(const on_observation_callback_t& f)
	{
		callbacksOnObservation_.emplace_back(f);
	}

	/** Calls all registered callbacks: */
	void dispatchOnObservation(
		const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);

	/** @} */

	/** Connect to server, advertise topics and services, etc. per the world
	 * description loaded from XML file. */
	void connectToServer();

	mvsim::Client& commsClient() { return client_; }
	const mvsim::Client& commsClient() const { return client_; }

	void free_opengl_resources();

	auto& physical_objects_mtx() { return worldPhysicalMtx_; }

	bool headless() const { return guiOptions_.headless; }
	void headless(bool setHeadless) { guiOptions_.headless = setHeadless; }

	bool sensor_has_to_create_egl_context();

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

	float collisionThreshold() const {return collisionThreshold_;}

   private:
	friend class VehicleBase;
	friend class Block;

	mvsim::Client client_{"World"};

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
		{"save_ground_truth_trajectory",
		 {"%s", &save_ground_truth_trajectory_}},
		{"ground_truth_rate", {"%lf", &ground_truth_rate_}},
	};

	/** User-defined variables as defined via `<variable name='' value='' />`
	 * tags in the World xml file, for use within `$f{}` expressions */
	std::map<std::string, std::string> userDefinedVariables_;

	/** In seconds, real simulation time since beginning (may be different than
	 * wall-clock time because of time warp, etc.) */
	double simulTime_ = 0;
	std::optional<double> simul_start_wallclock_time_;
	std::mutex simul_time_mtx_;

	/** Path from which to take relative directories. */
	std::string basePath_{"."};

	/// This private container will be filled with objects in the public
	/// gui_user_objects_
	mrpt::opengl::CSetOfObjects::Ptr glUserObjsPhysical_ =
		mrpt::opengl::CSetOfObjects::Create();
	mrpt::opengl::CSetOfObjects::Ptr glUserObjsViz_ =
		mrpt::opengl::CSetOfObjects::Create();

	// ------- GUI options -----
	struct TGUI_Options
	{
		unsigned int win_w = 800, win_h = 600;
		bool start_maximized = true;
		int refresh_fps = 20;
		bool ortho = false;
		bool show_forces = false;
		bool show_sensor_points = true;
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
		void parse_from(
			const rapidxml::xml_node<char>& node, COutputLogger& logger);
	};

	/** Some of these options are only used the first time the GUI window is
	 * created. */
	TGUI_Options guiOptions_;

	struct LightOptions
	{
		LightOptions() = default;

		void parse_from(
			const rapidxml::xml_node<char>& node, COutputLogger& logger);

		bool enable_shadows = true;
		int shadow_map_size = 2048;

		double light_azimuth = mrpt::DEG2RAD(45.0);
		double light_elevation = mrpt::DEG2RAD(70.0);

		float light_clip_plane_min = 0.1f;
		float light_clip_plane_max = 900.0f;

		float shadow_bias = 1e-5;
		float shadow_bias_cam2frag = 1e-5;
		float shadow_bias_normal = 1e-4;

		mrpt::img::TColor light_color = {0xff, 0xff, 0xff, 0xff};
		float light_ambient = 0.5f;

		float eye_distance_to_shadow_map_extension = 2.0f;	//!< [m/m]
		float minimum_shadow_map_extension_ratio = 0.005f;	//!< [0,1]

		const TParameterDefinitions params = {
			{"enable_shadows", {"%bool", &enable_shadows}},
			{"shadow_map_size", {"%i", &shadow_map_size}},
			{"light_azimuth_deg", {"%lf_deg", &light_azimuth}},
			{"light_elevation_deg", {"%lf_deg", &light_elevation}},
			{"light_clip_plane_min", {"%f", &light_clip_plane_min}},
			{"light_clip_plane_max", {"%f", &light_clip_plane_max}},
			{"light_color", {"%color", &light_color}},
			{"shadow_bias", {"%f", &shadow_bias}},
			{"shadow_bias_cam2frag", {"%f", &shadow_bias_cam2frag}},
			{"shadow_bias_normal", {"%f", &shadow_bias_normal}},
			{"light_ambient", {"%f", &light_ambient}},
			{"eye_distance_to_shadow_map_extension",
			 {"%f", &eye_distance_to_shadow_map_extension}},
			{"minimum_shadow_map_extension_ratio",
			 {"%f", &minimum_shadow_map_extension_ratio}},
		};
	};

	/** Options for lights */
	LightOptions lightOptions_;

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

	bool initialized_ = false;

	// List of all objects above (vehicles, world_elements, blocks), but as
	// shared_ptr to their Simulable interfaces, so we can easily iterate on
	// this list only for common tasks:
	SimulableList simulableObjects_;
	std::mutex simulableObjectsMtx_;

	/** Runs one individual time step */
	void internal_one_timestep(double dt);

	std::mutex simulationStepRunningMtx_;

	/** GUI stuff  */
	struct GUI
	{
		GUI(World& parent) : parent_(parent) {}

		mrpt::gui::CDisplayWindowGUI::Ptr gui_win;
		nanogui::Label* lbCpuUsage = nullptr;
		std::vector<nanogui::Label*> lbStatuses;
		nanogui::Button* btnReplaceObject = nullptr;

		struct InfoPerObject
		{
			nanogui::CheckBox* cb = nullptr;
			Simulable::Ptr simulable;
			VisualObject* visual = nullptr;
		};

		// Buttons that must be {dis,en}abled when there is a selected object:
		std::vector<nanogui::Widget*> btns_selectedOps;
		std::vector<InfoPerObject> gui_cbObjects;
		InfoPerObject gui_selectedObject;

		mrpt::math::TPoint3D clickedPt{0, 0, 0};

		void prepare_control_window();
		void prepare_status_window();
		void prepare_editor_window();

		void handle_mouse_operations();

	   private:
		World& parent_;
	};
	GUI gui_{*this};  //!< gui state

	/** 3D scene with all visual objects (vehicles, obstacles, markers, etc.)
	 *  \sa worldPhysical_
	 */
	mrpt::opengl::COpenGLScene::Ptr worldVisual_ =
		mrpt::opengl::COpenGLScene::Create();

	/** 3D scene with all physically observable objects: we will use this
	 * scene as input to simulated sensors like cameras, where we don't wont
	 * to see visualization marks, etc.
	 * \sa world_visual_
	 */
	mrpt::opengl::COpenGLScene worldPhysical_;
	std::recursive_mutex worldPhysicalMtx_;

	/// Updated in internal_one_step()
	std::map<std::string, mrpt::math::TPose3D> copy_of_objects_dynstate_pose_;
	std::map<std::string, mrpt::math::TTwist2D> copy_of_objects_dynstate_twist_;
	std::set<std::string> copy_of_objects_had_collision_;
	std::recursive_mutex copy_of_objects_dynstate_mtx_;

	std::set<std::string> reset_collision_flags_;
	std::mutex reset_collision_flags_mtx_;

	void internal_gui_on_observation(
		const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);
	void internal_gui_on_observation_3Dscan(
		const Simulable& veh,
		const std::shared_ptr<mrpt::obs::CObservation3DRangeScan>& obs);
	void internal_gui_on_observation_image(
		const Simulable& veh,
		const std::shared_ptr<mrpt::obs::CObservationImage>& obs);

	mrpt::math::TPoint2D internal_gui_on_image(
		const std::string& label, const mrpt::img::CImage& im, int winPosX);

	std::map<std::string, nanogui::Window*> guiObsViz_;	 //!< by sensorLabel

	/** Changes the light source direction from azimuth and elevation angles (in
	 * radians) */
	void setLightDirectionFromAzimuthElevation(
		const float azimuth, const float elevation);

	/** @} */  // end GUI stuff

	mrpt::system::CTimeLogger timlogger_{true /*enabled*/, "mvsim::World"};
	mrpt::system::CTicTac timer_iteration_;

	void process_load_walls(const rapidxml::xml_node<char>& node);
	void insertBlock(const Block::Ptr& block);

	struct XmlParserContext
	{
		XmlParserContext(
			const rapidxml::xml_node<char>* n, const std::string& basePath)
			: node(n), currentBasePath(basePath)
		{
		}

		const rapidxml::xml_node<char>* node = nullptr;
		const std::string currentBasePath;
	};

	/// This will parse a main XML file, or its included
	void internal_recursive_parse_XML(const XmlParserContext& ctx);

	using xml_tag_parser_function_t =
		std::function<void(const XmlParserContext&)>;

	std::map<std::string, xml_tag_parser_function_t> xmlParsers_;

	void register_standard_xml_tag_parsers();

	void register_tag_parser(
		const std::string& xmlTagName, const xml_tag_parser_function_t& f)
	{
		xmlParsers_.emplace(xmlTagName, f);
	}
	void register_tag_parser(
		const std::string& xmlTagName,
		void (World::*f)(const XmlParserContext& ctx))
	{
		xmlParsers_.emplace(
			xmlTagName,
			[this, f](const XmlParserContext& ctx) { (this->*f)(ctx); });
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
	void parse_tag_walls(const XmlParserContext& ctx);
	void parse_tag_include(const XmlParserContext& ctx);
	void parse_tag_variable(const XmlParserContext& ctx);
	void parse_tag_for(const XmlParserContext& ctx);
	void parse_tag_if(const XmlParserContext& ctx);

	// ======== end of XML parser tags ========

	mutable RemoteResourcesManager remoteResources_;

	void internalOnObservation(
		const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);

	void internalPostSimulStepForRawlog();
	void internalPostSimulStepForTrajectory();

	std::mutex rawlog_io_mtx_;
	std::map<std::string, std::shared_ptr<mrpt::io::CFileGZOutputStream>>
		rawlog_io_per_veh_;
	std::optional<double> rawlog_last_odom_time_;

	std::mutex gt_io_mtx_;
	std::map<std::string, std::fstream> gt_io_per_veh_;
	std::optional<double> gt_last_time_;

	// Services:
	void internal_advertiseServices();	// called from connectToServer()

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

	mvsim_msgs::SrvSetPoseAnswer srv_set_pose(
		const mvsim_msgs::SrvSetPose& req);
	mvsim_msgs::SrvGetPoseAnswer srv_get_pose(
		const mvsim_msgs::SrvGetPose& req);
	mvsim_msgs::SrvSetControllerTwistAnswer srv_set_controller_twist(
		const mvsim_msgs::SrvSetControllerTwist& req);
	mvsim_msgs::SrvShutdownAnswer srv_shutdown(
		const mvsim_msgs::SrvShutdown& req);
#endif
};
}  // namespace mvsim
