/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/img/TColor.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Block.h>
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

	double get_simul_time() const
	{
		return m_simul_time;
	}  //!< Simulation wall-clock time

	double get_simul_timestep() const
	{
		return m_simul_timestep;
	}  //!< Simulation fixed-time interval for numerical integration
	void set_simul_timestep(double timestep)
	{
		m_simul_timestep = timestep;
	}  //!< Simulation fixed-time interval for numerical integration

	double get_gravity() const
	{
		return m_gravity;
	}  //!< Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights
	   //! for friction, etc.
	void set_gravity(double accel)
	{
		m_gravity = accel;
	}  //!< Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights
	   //! for friction, etc.

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

	void internalUpdate3DSceneObjects(
		mrpt::opengl::COpenGLScene::Ptr& gl_scene);
	void internal_GUI_thread();

	std::string m_gui_msg_lines;
	std::mutex m_gui_msg_lines_mtx;

	std::thread m_gui_thread;

	std::atomic_bool m_gui_thread_running = false;
	std::atomic_bool m_gui_thread_must_close = false;
	std::mutex m_gui_thread_start_mtx;

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
	using TListVehicles = std::multimap<std::string, VehicleBase*>;

	/** See getListOfWorldElements() */
	using TListWorldElements = std::list<WorldElementBase*>;

	/** Map 'block-name' => block object. See getListOfBlocks()*/
	using TListBlocks = std::multimap<std::string, Block*>;

	/** @} */

	/** \name Access inner working objects
	  @{*/
	std::unique_ptr<b2World>& getBox2DWorld() { return m_box2d_world; }
	const std::unique_ptr<b2World>& getBox2DWorld() const
	{
		return m_box2d_world;
	}
	b2Body* getBox2DGroundBody() { return m_b2_ground_body; }
	const TListVehicles& getListOfVehicles() const { return m_vehicles; }
	TListVehicles& getListOfVehicles() { return m_vehicles; }
	const TListBlocks& getListOfBlocks() const { return m_blocks; }
	TListBlocks& getListOfBlocks() { return m_blocks; }
	const TListWorldElements& getListOfWorldElements() const
	{
		return m_world_elements;
	}

	mrpt::system::CTimeLogger& getTimeLogger() { return m_timlogger; }
	/** Replace macros, prefix the base_path if input filename is relative, etc.
	 */
	std::string resolvePath(const std::string& in_path) const;

	/** @} */

	/** \name Visitors API
	  @{*/
	/** Derive from this class to call runVisitorOnVehicles() */
	struct VehicleVisitorBase
	{
		virtual void visit(VehicleBase* obj) = 0;
	};
	/** Derive from this class to call runVisitorOnWorldElements() */
	struct WorldElementVisitorBase
	{
		virtual void visit(WorldElementBase* obj) = 0;
	};

	/** Run the user-provided visitor on each vehicle */
	void runVisitorOnVehicles(VehicleVisitorBase& v);

	/** Run the user-provided visitor on each world element */
	void runVisitorOnWorldElements(WorldElementVisitorBase& v);

	/** @} */

	/** \name Optional user hooks
	  @{*/
	virtual void onNewObservation(
		[[maybe_unused]] const VehicleBase& veh,
		[[maybe_unused]] const mrpt::obs::CObservation* obs)
	{
		/* default: do nothing */
	}
	/** @} */

   private:
	friend class VehicleBase;
	friend class Block;

	// -------- World Params ----------
	/** Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for
	 * friction, etc. */
	double m_gravity = 9.81;

	/** Simulation fixed-time interval for numerical integration.*/
	double m_simul_timestep = 10e-3;

	/** Velocity and position iteration count (refer to libbox2d docs) */
	int m_b2d_vel_iters = 6, m_b2d_pos_iters = 3;

	const TParameterDefinitions m_other_world_params = {
		{"gravity", {"%lf", &m_gravity}},
		{"simul_timestep", {"%lf", &m_simul_timestep}},
		{"b2d_vel_iters", {"%i", &m_b2d_vel_iters}},
		{"b2d_pos_iters", {"%i", &m_b2d_pos_iters}},
	};

	/** In seconds, real simulation time since beginning (may be different than
	 * wall-clock time because of time warp, etc.) */
	double m_simul_time = 0;

	/** Path from which to take relative directories. */
	std::string m_base_path{"."};

	// ------- GUI options -----
	struct TGUI_Options
	{
		unsigned int win_w = 800, win_h = 600;
		bool start_maximized = true;
		int refresh_fps = 20;
		bool ortho = false;
		bool show_forces = false;
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

	TListVehicles m_vehicles;
	TListWorldElements m_world_elements;
	TListBlocks m_blocks;

	/** Runs one individual time step */
	void internal_one_timestep(double dt);

	// -------- GUI stuff ----------
	mrpt::gui::CDisplayWindowGUI::Ptr m_gui_win;

	mrpt::system::CTimeLogger m_timlogger;
	mrpt::system::CTicTac m_timer_iteration;
};
}  // namespace mvsim
