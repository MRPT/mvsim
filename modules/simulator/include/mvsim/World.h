/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Block.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <list>

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
	double get_simul_time() const { return m_simul_time; }

	/** Get the current simulation full timestamp, computed as the
	 *  real wall clock timestamp at the beginning of the simulation,
	 *  plus the number of seconds simulation has run.
	 *  \sa get_simul_time()
	 */
	mrpt::Clock::time_point get_simul_timestamp() const
	{
		ASSERT_(m_simul_start_wallclock_time.has_value());
		return mrpt::Clock::fromDouble(
			m_simul_time + m_simul_start_wallclock_time.value());
	}

	/// Simulation fixed-time interval for numerical integration
	double get_simul_timestep() const { return m_simul_timestep; }
	/// Simulation fixed-time interval for numerical integration
	void set_simul_timestep(double timestep) { m_simul_timestep = timestep; }

	/// Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for
	/// friction, etc.
	double get_gravity() const { return m_gravity; }

	/// Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for
	/// friction, etc.
	void set_gravity(double accel) { m_gravity = accel; }

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
		return m_gui.gui_win;
	}

	const mrpt::math::TPoint3D& gui_mouse_point() const
	{
		return m_gui.clickedPt;
	}

	/** If !=null, a set of objects to be rendered merged with the default
	 * visualization. Lock the mutex m_gui_user_objects_mtx while writing.
	 * There are two sets of objects: "viz" for visualization only, "physical"
	 * for objects which should be detected by sensors.
	 */
	mrpt::opengl::CSetOfObjects::Ptr m_gui_user_objects_physical,
		m_gui_user_objects_viz;
	std::mutex m_gui_user_objects_mtx;

	void internalRunSensorsOn3DScene(
		mrpt::opengl::COpenGLScene& physicalObjects);

	void internalUpdate3DSceneObjects(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical);
	void internal_GUI_thread();
	void internal_process_pending_gui_user_tasks();

	std::mutex m_pendingRunSensorsOn3DSceneMtx;
	bool m_pendingRunSensorsOn3DScene = false;

	void mark_as_pending_running_sensors_on_3D_scene()
	{
		m_pendingRunSensorsOn3DSceneMtx.lock();
		m_pendingRunSensorsOn3DScene = true;
		m_pendingRunSensorsOn3DSceneMtx.unlock();
	}
	void clear_pending_running_sensors_on_3D_scene()
	{
		m_pendingRunSensorsOn3DSceneMtx.lock();
		m_pendingRunSensorsOn3DScene = false;
		m_pendingRunSensorsOn3DSceneMtx.unlock();
	}
	bool pending_running_sensors_on_3D_scene()
	{
		m_pendingRunSensorsOn3DSceneMtx.lock();
		bool ret = m_pendingRunSensorsOn3DScene;
		m_pendingRunSensorsOn3DSceneMtx.unlock();
		return ret;
	}

	std::string m_gui_msg_lines;
	std::mutex m_gui_msg_lines_mtx;

	std::thread m_gui_thread;

	std::atomic_bool m_gui_thread_running = false;
	std::atomic_bool m_gui_thread_must_close = false;
	std::mutex m_gui_thread_start_mtx;

	void enqueue_task_to_run_in_gui_thread(const std::function<void(void)>& f)
	{
		m_gui_user_pending_tasks_mtx.lock();
		m_gui_user_pending_tasks.emplace_back(f);
		m_gui_user_pending_tasks_mtx.unlock();
	}

	std::vector<std::function<void(void)>> m_gui_user_pending_tasks;
	std::mutex m_gui_user_pending_tasks_mtx;

	TGUIKeyEvent m_lastKeyEvent;
	std::atomic_bool m_lastKeyEventValid = false;
	std::mutex m_lastKeyEvent_mtx;

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
	std::unique_ptr<b2World>& getBox2DWorld() { return m_box2d_world; }
	const std::unique_ptr<b2World>& getBox2DWorld() const
	{
		return m_box2d_world;
	}
	b2Body* getBox2DGroundBody() { return m_b2_ground_body; }
	const VehicleList& getListOfVehicles() const { return m_vehicles; }
	VehicleList& getListOfVehicles() { return m_vehicles; }
	const BlockList& getListOfBlocks() const { return m_blocks; }
	BlockList& getListOfBlocks() { return m_blocks; }
	const WorldElementList& getListOfWorldElements() const
	{
		return m_world_elements;
	}

	SimulableList& getListOfSimulableObjects() { return m_simulableObjects; }
	const SimulableList& getListOfSimulableObjects() const
	{
		return m_simulableObjects;
	}

	mrpt::system::CTimeLogger& getTimeLogger() { return m_timlogger; }
	/** Replace macros, prefix the base_path if input filename is relative, etc.
	 */
	std::string resolvePath(const std::string& in_path) const;

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
		m_callbacksOnObservation.emplace_back(f);
	}

	/** Calls all registered callbacks: */
	void dispatchOnObservation(
		const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
	{
		for (const auto& cb : m_callbacksOnObservation) cb(veh, obs);
	}

	/** @} */

	/** Connect to server, advertise topics and services, etc. per the world
	 * description loaded from XML file. */
	void connectToServer();

	mvsim::Client& commsClient() { return m_client; }
	const mvsim::Client& commsClient() const { return m_client; }

	auto& physical_objects_mtx() { return m_physical_objects_mtx; }

   private:
	friend class VehicleBase;
	friend class Block;

	mvsim::Client m_client{"World"};

	std::vector<on_observation_callback_t> m_callbacksOnObservation;

	// -------- World Params ----------
	/** Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for
	 * friction, etc. */
	double m_gravity = 9.81;

	/** Simulation fixed-time interval for numerical integration.*/
	double m_simul_timestep = 10e-3;

	/** Velocity and position iteration count (refer to libbox2d docs) */
	int m_b2d_vel_iters = 6, m_b2d_pos_iters = 3;

	std::string m_server_address = "localhost";

	const TParameterDefinitions m_other_world_params = {
		{"server_address", {"%s", &m_server_address}},
		{"gravity", {"%lf", &m_gravity}},
		{"simul_timestep", {"%lf", &m_simul_timestep}},
		{"b2d_vel_iters", {"%i", &m_b2d_vel_iters}},
		{"b2d_pos_iters", {"%i", &m_b2d_pos_iters}},
	};

	/** In seconds, real simulation time since beginning (may be different than
	 * wall-clock time because of time warp, etc.) */
	double m_simul_time = 0;
	std::optional<double> m_simul_start_wallclock_time;

	/** Path from which to take relative directories. */
	std::string m_base_path{"."};

	/// This private container will be filled with objects in the public
	/// m_gui_user_objects
	mrpt::opengl::CSetOfObjects::Ptr m_glUserObjsPhysical =
		mrpt::opengl::CSetOfObjects::Create();
	mrpt::opengl::CSetOfObjects::Ptr m_glUserObjsViz =
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
		double fov_deg = 60.0;
		/** Name of the vehicle to follow (empty=none) */
		std::string follow_vehicle;

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
		};

		TGUI_Options() = default;
		void parse_from(const rapidxml::xml_node<char>& node);
	};

	TGUI_Options m_gui_options;	 //!< Some of these options are only used the
								 //! first time the GUI window is created.

	// -------- World contents ----------
	/** Mutex protecting simulation objects from multi-thread access */
	std::recursive_mutex m_world_cs;

	/** Box2D dynamic simulator instance */
	std::unique_ptr<b2World> m_box2d_world;

	/** Used to declare friction between vehicles-ground*/
	b2Body* m_b2_ground_body = nullptr;

	VehicleList m_vehicles;
	WorldElementList m_world_elements;
	BlockList m_blocks;

	// List of all objects above (vehicles, world_elements, blocks), but as
	// shared_ptr to their Simulable interfaces, so we can easily iterate on
	// this list only for common tasks:
	SimulableList m_simulableObjects;

	/** Runs one individual time step */
	void internal_one_timestep(double dt);

	std::mutex m_simulationStepRunningMtx;

	/** GUI stuff  */
	struct GUI
	{
		GUI(World& parent) : m_parent(parent) {}

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
		World& m_parent;
	};
	GUI m_gui{*this};  //!< gui state

	/// 3D scene with all physically observable objects: we will use this
	/// scene as input to simulated sensors like cameras, where we don't wont
	/// to see visualization marks, etc.
	mrpt::opengl::COpenGLScene m_physical_objects;
	std::recursive_mutex m_physical_objects_mtx;

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

	std::map<std::string, nanogui::Window*> m_gui_obs_viz;	//!< by sensorLabel

	/** @} */  // end GUI stuff

	mrpt::system::CTimeLogger m_timlogger{true /*enabled*/, "mvsim::World"};
	mrpt::system::CTicTac m_timer_iteration;

	void process_load_walls(const rapidxml::xml_node<char>& node);
	void insertBlock(const Block::Ptr& block);

	/// This will parse a main XML file, or its included
	void internal_recursive_parse_XML(
		const void* /*rapidxml::xml_node<>* */ node,
		const std::string& currentBasePath);
};
}  // namespace mvsim
