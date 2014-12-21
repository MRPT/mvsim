/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mvsim/VehicleBase.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/gui/CDisplayWindow3D.h>

#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>

#include <list>

namespace mvsim
{
	/** Simulation happens inside a World object.
	  * This is the central class for usage from user code, running the simulation,
	  * loading XML models, managing GUI visualization, etc.
	  * The ROS node acts as a bridge between this class and the ROS subsystem.
	  */
	class World
	{
	public:
		/** \name Initialization, simulation set-up
		  @{*/
		World(); //!< Default ctor: inits an empty world
		~World(); //!< Dtor.

		void clear_all(bool acquire_mt_lock=true); //!< Resets the entire simulation environment to an empty world. \a acquire_mt_lock determines whether to lock the multithreading semaphore before (set to false only if it's already acquired).

		/** Load an entire world description into this object from a specification in XML format.
		  * \param[in] fileNameForPath Optionally, provide the full path to an XML file from which to take relative paths.
		  * \exception std::exception On any error, with what() giving a descriptive error message
		  */
		void load_from_XML(const std::string &xml_text, const std::string &fileNameForPath=std::string("."));
		/** @} */

		/** \name Simulation execution
		  @{*/

		double get_simul_time() const { return m_simul_time; } //!< Simulation wall-clock time

		double get_simul_timestep() const { return m_simul_timestep; } //!< Simulation fixed-time interval for numerical integration
		void   set_simul_timestep(double timestep) { m_simul_timestep=timestep; } //!< Simulation fixed-time interval for numerical integration

		double get_gravity() const { return m_gravity; } //!< Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for friction, etc.
		void set_gravity(double accel) { m_gravity=accel; } //!< Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for friction, etc.

		/** Runs the simulation for a given time interval (in seconds)
		  * \note The minimum simulation time is the timestep set (e.g. via set_simul_timestep()), even if time advanced further than the provided "dt".
		  */
		void run_simulation(double dt);

		/** For usage in TUpdateGUIParams and \a update_GUI() */
		struct TGUIKeyEvent
		{
			int keycode; //!< 0=no Key. Otherwise, ASCII code.
			mrpt::gui::mrptKeyModifier key_modifier;

			TGUIKeyEvent() : keycode(0) {}
		};

		struct TUpdateGUIParams
		{
			TGUIKeyEvent keyevent; //!< Keystrokes in the window are returned here.
			std::string msg_lines; //!< Messages to show

			TUpdateGUIParams();
		};

		/** Updates (or sets-up upon first call) the GUI visualization of the scene.
		  * \param[inout] params Optional inputs/outputs to the GUI update process. See struct for details.
		  * \note This method is prepared to be called concurrently with the simulation, and doing so is recommended to assure a smooth multi-threading simulation.
		  */
		void update_GUI( TUpdateGUIParams *params=NULL );

		bool is_GUI_open() const; //!< Return true if the GUI window is open, after a previous call to update_GUI()

		void close_GUI(); //!< Forces closing the GUI window, if any.

		/** @} */

		/** \name Public types
		  @{*/
		typedef std::multimap<std::string,VehicleBase*> TListVehicles; //!< Map 'vehicle-name' => vehicle object. See getListOfVehicles()
		typedef std::list<WorldElementBase*> TListWorldElements; //!< See getListOfWorldElements()
		/** @} */

		/** \name Access inner working objects
		  @{*/
		b2World* getBox2DWorld() { return m_box2d_world; }
		const b2World* getBox2DWorld() const { return m_box2d_world; }

		b2Body* getBox2DGroundBody() { return m_b2_ground_body; }

		const TListVehicles & getListOfVehicles() const { return m_vehicles; }
        TListVehicles &       getListOfVehicles()       { return m_vehicles; }
        const TListWorldElements & getListOfWorldElements() const { return m_world_elements; }

		mrpt::utils::CTimeLogger & getTimeLogger() { return m_timlogger; }

		/** Replace macros, prefix the base_path if input filename is relative, etc.
		  */
		std::string resolvePath(const std::string &in_path) const;

		/** @} */

		/** \name Visitors API
		  @{*/
		/** Derive from this class to call runVisitorOnVehicles() */
		struct VehicleVisitorBase
		{
			virtual void visit(VehicleBase *obj) = 0;
		};
		/** Derive from this class to call runVisitorOnWorldElements() */
		struct WorldElementVisitorBase
		{
			virtual void visit(WorldElementBase *obj) = 0;
		};

		/** Run the user-provided visitor on each vehicle */
		void runVisitorOnVehicles(VehicleVisitorBase &v);

		/** Run the user-provided visitor on each world element */
		void runVisitorOnWorldElements(WorldElementVisitorBase &v);

		/** @} */

		/** \name Optional user hooks
		  @{*/
		virtual void onNewObservation(const VehicleBase &veh, const mrpt::slam::CObservation* obs) { /* default: do nothing */ }
		/** @} */

	private:
		friend class VehicleBase;

		// -------- World Params ----------
		double m_gravity; //!< Gravity acceleration (Default=9.8 m/s^2). Used to evaluate weights for friction, etc.
		double m_simul_time;    //!< In seconds, real simulation time since beginning (may be different than wall-clock time because of time warp, etc.)
		double m_simul_timestep; //!< Simulation fixed-time interval for numerical integration.
		int m_b2d_vel_iters, m_b2d_pos_iters; //!< Velocity and position iteration count (Box2D)
		std::string m_base_path; //!< Path from which to take relative directories.

		// ------- GUI options -----
		struct TGUI_Options
		{
			unsigned int win_w,win_h;
			bool   ortho;
			bool   show_forces;
			double force_scale; //!< In meters/Newton
			double camera_distance;
			double fov_deg;
			std::string follow_vehicle; //!< Name of the vehicle to follow (empty=none)

			TGUI_Options();
			void parse_from(const rapidxml::xml_node<char> &node);
		};

		TGUI_Options m_gui_options;  //!< Some of these options are only used the first time the GUI window is created.


		// -------- World contents ----------
		mrpt::synch::CCriticalSection m_world_cs; //!< The main semaphore to protect simulation objects from multithreading access.

		b2World* m_box2d_world; //!< Box2D dynamic simulator instance
		b2Body*  m_b2_ground_body;  //!< Used to declare friction between vehicles-ground


		TListVehicles      m_vehicles;
		TListWorldElements m_world_elements;

		/** Runs one individual time step */
		void internal_one_timestep(double dt);


		// -------- GUI stuff ----------
		mrpt::gui::CDisplayWindow3DPtr  m_gui_win;


		mrpt::utils::CTimeLogger m_timlogger;
		mrpt::utils::CTicTac     m_timer_iteration;

	};

}
