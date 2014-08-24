/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/mv2dsim.h>

#include <mrpt/system/os.h> // kbhit()
#include <mrpt/system/threads.h> // sleep()
#include <mrpt/utils/CTicTac.h>

#include <rapidxml_utils.hpp>
#include <iostream>

using namespace mv2dsim;

struct TThreadParams
{
	World *world;
	volatile bool closing;
	TThreadParams(): world(NULL), closing(false) {}
};
void thread_update_GUI(TThreadParams &thread_params);
World::TGUIKeyEvent gui_key_events;

int main(int argc, char **argv)
{
    try
	{
		if (argc!=2) return -1;

		World  world;

		// Load from XML:
		rapidxml::file<> fil_xml(argv[1]);
		world.load_from_XML( fil_xml.data(), argv[1] );

		// Launch GUI thread:
		TThreadParams thread_params;
		thread_params.world = &world;
		mrpt::system::TThreadHandle thGUI = mrpt::system::createThreadRef( thread_update_GUI, thread_params);

		// Run simulation:
		mrpt::utils::CTicTac timer;
		timer.Tac(); // Reset clock

		while (!mrpt::system::os::kbhit())
		{
			// Compute how much time has passed to simulate in real-time:
			const double cur_tim = timer.Tac();
			const double incr_tim = cur_tim - world.get_simul_time();

			if (incr_tim>=world.get_simul_timestep())
				world.run_simulation(incr_tim);
			mrpt::system::sleep(10);

#if 0
			{ // Test: Get vehicles speed:
				const World::TListVehicles & vehs = world.getListOfVehicles();
				const vec3 &vel = (*vehs.begin())->getVelocityLocal();
				printf("vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg\n", vel.vals[0], vel.vals[1], mrpt::utils::RAD2DEG(vel.vals[2]) );
			}
#endif
#if 1
			{ // Test: Differential drive: Control raw forces
				const World::TListVehicles & vehs = world.getListOfVehicles();
				ASSERT_(!vehs.empty())
				DynamicsDifferential *veh = dynamic_cast<DynamicsDifferential*>(vehs.begin()->second);
				ASSERT_(veh)
				DynamicsDifferential::ControllerBasePtr &cntrl_ptr = veh->getController();
				DynamicsDifferential::ControllerRawForces*cntrl = dynamic_cast<DynamicsDifferential::ControllerRawForces*>(cntrl_ptr.pointer());
				ASSERT_(cntrl)

				switch (gui_key_events.keycode)
				{
				case 'q': case 'Q':  cntrl->setpoint_wheel_force_l += 5.0;  break;
				case 'w': case 'W':  cntrl->setpoint_wheel_force_r += 5.0;  break;
				case 'a': case 'A':  cntrl->setpoint_wheel_force_l -= 5.0;  break;
				case 's': case 'S':  cntrl->setpoint_wheel_force_r -= 5.0;  break;
				case ' ':cntrl->setpoint_wheel_force_r = 0.0; cntrl->setpoint_wheel_force_l=0.0;  break;
				};
				gui_key_events = World::TGUIKeyEvent(); // Reset key-stroke
			}
#endif

		}

		thread_params.closing = true;
		mrpt::system::joinThread( thGUI );

    } catch (const std::exception& e)
	{
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}


void thread_update_GUI(TThreadParams &thread_params)
{
	while (!thread_params.closing)
	{
		World::TGUIKeyEvent key;
		thread_params.world->update_GUI(&key);

		// Send key-strokes to the main thread:
		if(key.keycode!=0)
			gui_key_events = key;

		mrpt::system::sleep(25);
	}
}
