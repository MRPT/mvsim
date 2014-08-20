/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/World.h>
#include <mv2dsim/WorldElements/OccupancyGridMap.h>
#include <mv2dsim/Sensors/LaserScanner.h>
#include <mv2dsim/VehicleBase.h>
#include <mrpt/opengl/COpenGLScene.h>

#include "xml_utils.h"

using namespace mv2dsim;
using namespace rapidxml;

int z_order_cnt = 0;

LaserScanner::LaserScanner(VehicleBase&parent,const rapidxml::xml_node<char> *root) :
	m_z_order(++z_order_cnt),
	SensorBase(parent)
{
	this->loadConfigFrom(root);
}

LaserScanner::~LaserScanner()
{
}

void LaserScanner::loadConfigFrom(const rapidxml::xml_node<char> *root)
{
	m_gui_uptodate=false;

	// <pose>X Y YAW</pose>
	{
		xml_node<> *node = root->first_node("pose");
		if (node && node->value())
		{
			vec3 v=parseXYPHI( node->value() );
			const mrpt::poses::CPose2D p( v.vals[0],v.vals[1],v.vals[2] );
			m_scan_model.sensorPose = mrpt::poses::CPose3D(p);
		}
	}

	// Attribs:	
	TXMLAttribs attribs[] = {
     	{ "name","%s", &this->m_name }
	};
	parse_xmlnode_attribs(*root, attribs, sizeof(attribs)/sizeof(attribs[0]),"[LaserScanner]" );


	// Other scalar params:
	int nRays = 181;
	double fov_deg = 180;

	std::map<std::string,TParamEntry> params;
	params["fov_degrees"] = TParamEntry("%lf",&fov_deg);
	params["nrays"] = TParamEntry("%i",&nRays);

	// Parse XML:
	{
		xml_node<> *node = root->first_node();
		while (node)
		{
			parse_xmlnode_as_param(*node,params);

			// Move on to next node:
			node = node->next_sibling(NULL);
		}
	}

	// Pass params to the scan2D obj:
	m_scan_model.aperture = mrpt::utils::DEG2RAD(fov_deg);
	m_scan_model.scan.resize(nRays);
	m_scan_model.validRange.resize(nRays);

}

void LaserScanner::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	// 1st time?
	if (!m_gl_scan)
	{
		m_gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
		scene.insert(m_gl_scan);
	}

	if (!m_gui_uptodate)
	{
		{
			mrpt::synch::CCriticalSectionLocker csl(&m_last_scan_cs);
			if (m_last_scan2gui) {
				m_gl_scan->setScan( *m_last_scan2gui );
				m_last_scan2gui.clear_unique();
			}
		}
		m_gui_uptodate=true;
	}

	const double z_incrs = 10e-3; //for m_z_order
	const mrpt::poses::CPose2D &p = m_vehicle.getCPose2D();
	m_gl_scan->setPose( mrpt::poses::CPose3D(p.x(),p.y(), z_incrs*m_z_order, p.phi(), 0.0, 0.0 ) );
}

void LaserScanner::simul_pre_timestep(const TSimulContext &context)
{
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void LaserScanner::simul_post_timestep(const TSimulContext &context)
{
	// Create an array of scans, each reflecting ranges to one kind of world objects. 
	// Finally, we'll take the shortest range in each direction:
	std::list<mrpt::slam::CObservation2DRangeScan> lstScans;

	// Get pose of the robot:
	const mrpt::poses::CPose2D &vehPose = m_vehicle.getCPose2D();
	
	const double rangeStdNoise = 0.001;
	const double angleStdNoise = mrpt::utils::DEG2RAD(0.01);
	
	// grid maps:
	// -------------
	const World::TListWorldElements &elements = m_world->getListOfWorldElements();
	for (World::TListWorldElements::const_iterator it=elements.begin();it!=elements.end();++it)
	{
		// If not a grid map, ignore:
		const OccupancyGridMap* grid = dynamic_cast<const OccupancyGridMap*>(*it);
		if (!grid) continue;
		const mrpt::slam::COccupancyGridMap2D &occGrid = grid->getOccGrid();

		// Create new scan:
		lstScans.push_back( mrpt::slam::CObservation2DRangeScan(m_scan_model) );
		mrpt::slam::CObservation2DRangeScan &scan = lstScans.back();

		// Ray tracing over the gridmap:
		occGrid.laserScanSimulator(scan,vehPose,0.5f, m_scan_model.scan.size(), rangeStdNoise, 1, angleStdNoise);
	}
	
	// ray trace on Box2D polygons:
	// ------------------------------
	MRPT_TODO("Ray trace box2d")

	// Summarize all scans in one single scan:
	// ----------------------------------------
	mrpt::slam::CObservation2DRangeScan *lastScan = new mrpt::slam::CObservation2DRangeScan(m_scan_model);
	lastScan->timestamp = mrpt::system::now();
	
	const size_t nRays = lastScan->scan.size();
	const double maxRange = lastScan->maxRange;
	lastScan->scan.assign(nRays,maxRange);
	lastScan->validRange.assign(nRays, 0);

	for (std::list<mrpt::slam::CObservation2DRangeScan>::const_iterator it=lstScans.begin();it!=lstScans.end();++it)
	{
		ASSERT_(it->scan.size()==nRays && it->validRange.size()==nRays)

		for (size_t i=0;i<nRays;i++)
		{
			if (it->validRange[i])
			{
				lastScan->scan[i] = std::min(lastScan->scan[i], it->scan[i]);
				lastScan->validRange[i] = 1; // valid
			}
		}
	}

	{
		mrpt::synch::CCriticalSectionLocker csl(&m_last_scan_cs);
		m_last_scan = mrpt::slam::CObservation2DRangeScanPtr( lastScan );
		m_last_scan2gui = m_last_scan;
	}
	m_gui_uptodate = false;

}
