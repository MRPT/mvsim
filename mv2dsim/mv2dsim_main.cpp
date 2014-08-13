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

struct TThreadParams
{
	mv2dsim::World *world;
	volatile bool closing;
	TThreadParams(): world(NULL), closing(false) {}
};
void thread_update_GUI(TThreadParams &thread_params);
	
int main(int argc, char **argv) 
{
    try 
	{
		if (argc!=2) return -1;

		mv2dsim::World  world;
		
		// Load from XML:
		rapidxml::file<> fil_xml(argv[1]);
		world.load_from_XML( fil_xml.data() );
	
		// Launch GUI thread:
		TThreadParams thread_params;
		thread_params.world = &world;
		mrpt::system::TThreadHandle thGUI = mrpt::system::createThreadRef( thread_update_GUI, thread_params);

		// Run simulation:
		mrpt::utils::CTicTac timer;
		double last_tim = timer.Tac();

		while (!mrpt::system::os::kbhit())
		{
			// Compute how much time has passed to simulate in real-time:
			const double cur_tim = timer.Tac();
			const double incr_tim = cur_tim - last_tim;
			last_tim=cur_tim;
			
			world.run_simulation(incr_tim);
			mrpt::system::sleep(10);
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
		thread_params.world->update_GUI();
		mrpt::system::sleep(25);
	}
}