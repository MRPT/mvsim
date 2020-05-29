/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mutex>
#include <mvsim/Sensors/SensorBase.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x130
#include <mrpt/obs/CObservation2DRangeScan.h>
using mrpt::obs::CObservation2DRangeScan;
#else
#include <mrpt/slam/CObservation2DRangeScan.h>
using mrpt::slam::CObservation2DRangeScan;
using mrpt::slam::CObservation2DRangeScanPtr;
#endif

namespace mvsim
{
class LaserScanner : public SensorBase
{
	DECLARES_REGISTER_SENSOR(LaserScanner)
   public:
	LaserScanner(VehicleBase& parent, const rapidxml::xml_node<char>* root);
	virtual ~LaserScanner();

	virtual void loadConfigFrom(
		const rapidxml::xml_node<char>* root);  //!< See docs in base class
	virtual void gui_update(
		mrpt::opengl::COpenGLScene& scene);  //!< See docs in base class

	virtual void simul_pre_timestep(
		const TSimulContext& context);  //!< See docs in base class
	virtual void simul_post_timestep(
		const TSimulContext& context);  //!< See docs in base class

   protected:
	int m_z_order;  //!< to help rendering multiple scans
	mrpt::poses::CPose2D m_sensor_pose_on_veh;
	std::string m_name;  //!< sensor label/name
	double m_rangeStdNoise;
	double m_angleStdNoise;
	bool m_see_fixtures;  //!< Whether all box2d "fixtures" are visible (solid)
						  //!or not (Default=true)

	CObservation2DRangeScan m_scan_model;  //!< Store here all scan parameters.
										   //!This obj will be copied as a
										   //!"pattern" to fill it with actual
										   //!scan data.

	std::mutex m_last_scan_cs;
	mrpt::obs::CObservation2DRangeScan::Ptr
		m_last_scan;  //!< Last simulated scan
	mrpt::obs::CObservation2DRangeScan::Ptr m_last_scan2gui;

	bool m_gui_uptodate;  //!< Whether m_gl_scan has to be updated upon next
						  //!call of gui_update() from m_last_scan2gui
	mrpt::opengl::CPlanarLaserScan::Ptr m_gl_scan;
};
}
