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
		mrpt::utils::CTicTac tictac;
		double t_old = tictac.Tac();
		double t_old_simul = t_old;
		double REALTIME_FACTOR = 1.0;

		while (!mrpt::system::os::kbhit())
		{
			// Compute how much time has passed to simulate in real-time:
			double t_new = tictac.Tac();
			double incr_time = REALTIME_FACTOR * (t_new-t_old);

			if (incr_time >= world.get_simul_timestep())  // Just in case the computer is *really fast*...
			{
				// Simulate:
				world.run_simulation(incr_time);

				t_old_simul = world.get_simul_time();
				t_old = t_new;
			}

			mrpt::system::sleep(10);

#if 0
			{ // Test: Get vehicles speed:
				const World::TListVehicles & vehs = world.getListOfVehicles();
				const vec3 &vel = vehs.begin()->second->getVelocityLocal();
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
				DynamicsDifferential::ControllerTwistPI *cntrl = dynamic_cast<DynamicsDifferential::ControllerTwistPI*>(cntrl_ptr.pointer());
				if (cntrl)
				{
					switch (gui_key_events.keycode)
					{
					case 'w':  cntrl->setpoint_lin_speed += 0.1;  break;
					case 's':  cntrl->setpoint_lin_speed -= 0.1;  break;
					case 'a':  cntrl->setpoint_ang_speed += 5.0*M_PI/180;  break;
					case 'd':  cntrl->setpoint_ang_speed -= 5.0*M_PI/180;  break;
					case ' ':  cntrl->setpoint_lin_speed = 0.0; cntrl->setpoint_ang_speed=0.0;  break;
					};
					//printf("setpoint: lin=%.03f ang=%.03f deg\n", cntrl->setpoint_lin_speed, 180.0/M_PI*cntrl->setpoint_ang_speed);
				}
				DynamicsDifferential::ControllerRawForces *cntrl2 = dynamic_cast<DynamicsDifferential::ControllerRawForces*>(cntrl_ptr.pointer());
				if (cntrl2)
				{
					switch (gui_key_events.keycode)
					{
					case 'q':  cntrl2->setpoint_wheel_torque_l += 0.5; break;
					case 'a':  cntrl2->setpoint_wheel_torque_l -= 0.5; break;
					case 'e':  cntrl2->setpoint_wheel_torque_r += 0.5; break;
					case 'd':  cntrl2->setpoint_wheel_torque_r -= 0.5; break;
					case ' ': cntrl2->setpoint_wheel_torque_l = cntrl2->setpoint_wheel_torque_r = 0.0; break;
					};
					//printf("setpoint: tl=%.03f tr=%.03f deg\n", cntrl2->setpoint_wheel_torque_l, cntrl2->setpoint_wheel_torque_r);
				}
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
