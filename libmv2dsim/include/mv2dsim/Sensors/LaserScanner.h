/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/Sensors/SensorBase.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

namespace mv2dsim
{
	class LaserScanner : public SensorBase
	{
		DECLARES_REGISTER_SENSOR(LaserScanner)
	public:
		LaserScanner(VehicleBase&parent,const rapidxml::xml_node<char> *root);
		virtual ~LaserScanner();

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) ; //!< See docs in base class
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene); //!< See docs in base class

		virtual void simul_pre_timestep(const TSimulContext &context); //!< See docs in base class
		virtual void simul_post_timestep(const TSimulContext &context); //!< See docs in base class

	protected:
		mrpt::poses::CPose2D m_sensor_pose_on_veh;

		mrpt::slam::CObservation2DRangeScan m_scan_model; //!< Store here all scan parameters. This obj will be copied as a "pattern" to fill it with actual scan data.

		bool m_gui_uptodate; //!< Whether m_gl_scan has to be updated upon next call of gui_update()
		mrpt::opengl::CPlanarLaserScanPtr m_gl_scan;

	};
}
