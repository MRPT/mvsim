/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>  // kbhit()
#include <mvsim/mvsim.h>
#include <chrono>
#include <iostream>
#include <rapidxml_utils.hpp>
#include <thread>

using namespace mvsim;

struct TThreadParams
{
	World* world;
	volatile bool closing;
	TThreadParams() : world(NULL), closing(false) {}
};
void usage(const char* argv0);
void thread_update_GUI(TThreadParams& thread_params);
World::TGUIKeyEvent gui_key_events;
std::string msg2gui;

int main(int argc, char** argv)
{
	try
	{
		if (argc != 2)
		{
			usage(argv[0]);
			return -1;
		}

		World world;

		// Load from XML:
		rapidxml::file<> fil_xml(argv[1]);
		world.load_from_XML(fil_xml.data(), argv[1]);

		// Launch GUI thread:
		TThreadParams thread_params;
		thread_params.world = &world;
		std::thread thGUI =
			std::thread(thread_update_GUI, std::ref(thread_params));

		// Run simulation:
		mrpt::system::CTicTac tictac;
		double t_old = tictac.Tac();
		double REALTIME_FACTOR = 1.0;
		bool do_exit = false;
		size_t teleop_idx_veh = 0;  // Index of the vehicle to teleop

		while (!do_exit && !mrpt::system::os::kbhit())
		{
			// Simulation
			// ============================================================
			// Compute how much time has passed to simulate in real-time:
			double t_new = tictac.Tac();
			double incr_time = REALTIME_FACTOR * (t_new - t_old);

			if (incr_time >= world.get_simul_timestep())  // Just in case the
														  // computer is *really
														  // fast*...
			{
				// Simulate:
				world.run_simulation(incr_time);

				// t_old_simul = world.get_simul_time();
				t_old = t_new;
			}

			// I could use 10ms here but chono literals are since gcc 4.9.3
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

			// GUI msgs, teleop, etc.
			// ====================================================

			std::string txt2gui_tmp;
			World::TGUIKeyEvent keyevent = gui_key_events;

			// Global keys:
			switch (keyevent.keycode)
			{
				case 27:
					do_exit = true;
					break;
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
					teleop_idx_veh = keyevent.keycode - '1';
					break;
			};

			{  // Test: Differential drive: Control raw forces
				const World::TListVehicles& vehs = world.getListOfVehicles();
				txt2gui_tmp += mrpt::format(
					"Selected vehicle: %u/%u\n",
					static_cast<unsigned>(teleop_idx_veh + 1),
					static_cast<unsigned>(vehs.size()));
				if (vehs.size() > teleop_idx_veh)
				{
					// Get iterator to selected vehicle:
					World::TListVehicles::const_iterator it_veh = vehs.begin();
					std::advance(it_veh, teleop_idx_veh);

					// Get speed: ground truth
					{
						const vec3& vel = it_veh->second->getVelocityLocal();
						txt2gui_tmp += mrpt::format(
							"gt. vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n",
							vel.vals[0], vel.vals[1],
							mrpt::RAD2DEG(vel.vals[2]));
					}
					// Get speed: ground truth
					{
						const vec3& vel =
							it_veh->second->getVelocityLocalOdoEstimate();
						txt2gui_tmp += mrpt::format(
							"odo vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n",
							vel.vals[0], vel.vals[1],
							mrpt::RAD2DEG(vel.vals[2]));
					}

					// Generic teleoperation interface for any controller that
					// supports it:
					{
						ControllerBaseInterface* controller =
							it_veh->second->getControllerInterface();
						ControllerBaseInterface::TeleopInput teleop_in;
						ControllerBaseInterface::TeleopOutput teleop_out;
						teleop_in.keycode = keyevent.keycode;
						controller->teleop_interface(teleop_in, teleop_out);
						txt2gui_tmp += teleop_out.append_gui_lines;
					}
				}
			}

			// Clear the keystroke buffer
			if (keyevent.keycode != 0) gui_key_events = World::TGUIKeyEvent();

			msg2gui = txt2gui_tmp;  // send txt msgs to show in the GUI

		}  // end while()

		thread_params.closing = true;
		thGUI.join();  // TODO: It could break smth
	}
	catch (const std::exception& e)
	{
		std::cerr << "ERROR: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}

void thread_update_GUI(TThreadParams& thread_params)
{
	while (!thread_params.closing)
	{
		World::TUpdateGUIParams guiparams;
		guiparams.msg_lines = msg2gui;

		thread_params.world->update_GUI(&guiparams);

		// Send key-strokes to the main thread:
		if (guiparams.keyevent.keycode != 0)
			gui_key_events = guiparams.keyevent;

		std::this_thread::sleep_for(std::chrono::milliseconds(25));
	}
}

void usage(const char* argv0)
{
	std::cerr << "Usage:\n"
				 " "
			  << argv0 << " [WORLD_MODEL.xml]\n";
}
