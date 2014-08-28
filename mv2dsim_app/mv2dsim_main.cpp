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
void usage(const char*argv0);
void thread_update_GUI(TThreadParams &thread_params);
World::TGUIKeyEvent gui_key_events;
std::string msg2gui;

int main(int argc, char **argv)
{
	try
	{
		if (argc!=2) { usage( argv[0] ); return -1; }

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
		double REALTIME_FACTOR = 1.0;
		bool do_exit = false;
		size_t teleop_idx_veh = 0; // Index of the vehicle to teleop

		while (!do_exit && !mrpt::system::os::kbhit())
		{
			// Simulation ============================================================
			// Compute how much time has passed to simulate in real-time:
			double t_new = tictac.Tac();
			double incr_time = REALTIME_FACTOR * (t_new-t_old);

			if (incr_time >= world.get_simul_timestep())  // Just in case the computer is *really fast*...
			{
				// Simulate:
				world.run_simulation(incr_time);

				//t_old_simul = world.get_simul_time();
				t_old = t_new;
			}

			mrpt::system::sleep(10);


			// GUI msgs, teleop, etc. ====================================================

			std::string txt2gui_tmp;
			World::TGUIKeyEvent keyevent = gui_key_events;

			// Global keys:
			switch (keyevent.keycode)
			{
			case 27: do_exit=true; break;
			case '1': case '2': case '3': case '4': case '5': case '6':
				teleop_idx_veh = keyevent.keycode-'1';
				break;
			};

			{ // Test: Differential drive: Control raw forces
				const World::TListVehicles & vehs = world.getListOfVehicles();
				txt2gui_tmp+=mrpt::format("Selected vehicle: %u/%u\n", static_cast<unsigned>(teleop_idx_veh+1),static_cast<unsigned>(vehs.size()) );
				if (vehs.size()>teleop_idx_veh)
				{
					// Get iterator to selected vehicle:
					World::TListVehicles::const_iterator it_veh = vehs.begin();
					std::advance(it_veh, teleop_idx_veh);

					// Get speed:
					{
						const vec3 &vel = it_veh->second->getVelocityLocal();
						txt2gui_tmp+=mrpt::format("vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg\n", vel.vals[0], vel.vals[1], mrpt::utils::RAD2DEG(vel.vals[2]) );
					}

					DynamicsDifferential *veh_diff = dynamic_cast<DynamicsDifferential*>(it_veh->second);
					if (veh_diff)
					{
						DynamicsDifferential::ControllerBasePtr &cntrl_ptr = veh_diff->getController();
						DynamicsDifferential::ControllerTwistPID *cntrl = dynamic_cast<DynamicsDifferential::ControllerTwistPID*>(cntrl_ptr.pointer());
						if (cntrl)
						{
							switch (keyevent.keycode)
							{
							case 'w':  cntrl->setpoint_lin_speed += 0.1;  break;
							case 's':  cntrl->setpoint_lin_speed -= 0.1;  break;
							case 'a':  cntrl->setpoint_ang_speed += 2.0*M_PI/180;  break;
							case 'd':  cntrl->setpoint_ang_speed -= 2.0*M_PI/180;  break;
							case ' ':  cntrl->setpoint_lin_speed = 0.0; cntrl->setpoint_ang_speed=0.0;  break;
							};
							txt2gui_tmp+="[Controller=twist_pid] Teleop keys: w/s=forward/backward. a/d=left/right. spacebar=stop.\n";
							txt2gui_tmp+=mrpt::format("setpoint: lin=%.03f ang=%.03f deg\n", cntrl->setpoint_lin_speed, 180.0/M_PI*cntrl->setpoint_ang_speed);
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
							txt2gui_tmp+="[Controller=raw] Teleop keys: q/a=incr/decr left torque. e/d=incr/decr right torque. spacebar=stop.\n";
							txt2gui_tmp+=mrpt::format("setpoint: tl=%.03f tr=%.03f deg\n", cntrl2->setpoint_wheel_torque_l, cntrl2->setpoint_wheel_torque_r);
						}
					}
					DynamicsAckermann *veh_ack = dynamic_cast<DynamicsAckermann*>(it_veh->second);
					if (veh_ack)
					{
						DynamicsAckermann::ControllerBasePtr &cntrl_ptr = veh_ack->getController();
						DynamicsAckermann::ControllerRawForces *cntrl2 = dynamic_cast<DynamicsAckermann::ControllerRawForces*>(cntrl_ptr.pointer());
						if (cntrl2)
						{
							switch (gui_key_events.keycode)
							{
							case 'w':  cntrl2->setpoint_wheel_torque_l-= 1.0; cntrl2->setpoint_wheel_torque_r-=1.0; break;
							case 's':  cntrl2->setpoint_wheel_torque_l+= 1.0; cntrl2->setpoint_wheel_torque_r+=1.0; break;
							case 'a':  cntrl2->setpoint_steer_ang += 1.0*M_PI/180.0; mrpt::utils::keep_min(cntrl2->setpoint_steer_ang, veh_ack->getMaxSteeringAngle()); break;
							case 'd':  cntrl2->setpoint_steer_ang -= 1.0*M_PI/180.0; mrpt::utils::keep_max(cntrl2->setpoint_steer_ang, -veh_ack->getMaxSteeringAngle()); break;
							case ' ':  cntrl2->setpoint_wheel_torque_l= .0; cntrl2->setpoint_wheel_torque_r=.0; break;
							};
							txt2gui_tmp+="[Controller=raw] Teleop keys: w/s=incr/decr torques. a/d=left/right steering. spacebar=stop.\n";
							txt2gui_tmp+=mrpt::format("setpoint: t=%.03f steer=%.03f deg\n", cntrl2->setpoint_wheel_torque_l, cntrl2->setpoint_steer_ang*180.0/M_PI);
						}
					}
				}
			}

			// Clear the keystroke buffer
			if (keyevent.keycode!=0)
				gui_key_events = World::TGUIKeyEvent();

			msg2gui = txt2gui_tmp;  // send txt msgs to show in the GUI

		} // end while()

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
		World::TUpdateGUIParams guiparams;
		guiparams.msg_lines = msg2gui;

		thread_params.world->update_GUI(&guiparams);

		// Send key-strokes to the main thread:
		if(guiparams.keyevent.keycode!=0)
			gui_key_events = guiparams.keyevent;

		mrpt::system::sleep(25);
	}
}

void usage(const char*argv0)
{
	std::cerr << 
		"Usage:\n"
		" " << argv0 << " [WORLD_MODEL.xml]\n";
}