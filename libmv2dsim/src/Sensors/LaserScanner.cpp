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
#include <mrpt/random.h>

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

	const double z_incrs  = 10e-3; //for m_z_order
	const double z_offset = 10e-2; 
	const mrpt::poses::CPose2D &p = m_vehicle.getCPose2D();
	m_gl_scan->setPose( mrpt::poses::CPose3D(p.x(),p.y(), z_offset+z_incrs*m_z_order, p.phi(), 0.0, 0.0 ) );
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

	const size_t nRays = m_scan_model.scan.size();
	const double maxRange = m_scan_model.maxRange;

	// Get pose of the robot:
	const mrpt::poses::CPose2D &vehPose = m_vehicle.getCPose2D();
	
	const double rangeStdNoise = 0.01;
	const double angleStdNoise = mrpt::utils::DEG2RAD(0.01);
	
	// grid maps:
	// -------------
	m_world->getTimeLogger().enter("LaserScanner.scan.1.gridmap");

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
	m_world->getTimeLogger().leave("LaserScanner.scan.1.gridmap");
	
	// ray trace on Box2D polygons:
	// ------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.2.polygons");
	{
		// Create new scan:
		lstScans.push_back( mrpt::slam::CObservation2DRangeScan(m_scan_model) );
		mrpt::slam::CObservation2DRangeScan &scan = lstScans.back();

		// Do Box2D raycasting stuff:
		// ------------------------------
		// This callback finds the closest hit. Polygon 0 is filtered.
		class RayCastClosestCallback : public b2RayCastCallback
		{
		public:
			RayCastClosestCallback()
			{
				m_hit = false;
			}

			float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
			{
				m_hit = true;
				m_point = point;
				m_normal = normal;
				// By returning the current fraction, we instruct the calling code to clip the ray and
				// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
				// are reported in order. However, by clipping, we can always get the closest fixture.
				return fraction;
			}
	
			bool m_hit;
			b2Vec2 m_point;
			b2Vec2 m_normal;
		};

		const mrpt::poses::CPose2D sensorPose = vehPose + mrpt::poses::CPose2D(scan.sensorPose);
		const b2Vec2 sensorPt = b2Vec2(sensorPose.x(),sensorPose.y());

		RayCastClosestCallback callback;

		// Scan size:
		scan.scan.resize(nRays);
		scan.validRange.resize(nRays);

		double  A = sensorPose.phi() + (scan.rightToLeft ? -0.5:+0.5) *scan.aperture;
		const double AA = (scan.rightToLeft ? 1.0:-1.0) * (scan.aperture / nRays);

		for (size_t i=0;i<nRays;i++,A+=AA)
		{
			const b2Vec2 endPt = b2Vec2(sensorPt.x + cos(A)*maxRange,sensorPt.y + sin(A)*maxRange);
			
			callback.m_hit=false;
			m_world->getBox2DWorld()->RayCast(&callback, sensorPt, endPt);
			scan.validRange[i] = callback.m_hit ? 1:0;
			
			float &range = scan.scan[i];
			if (callback.m_hit)
			{
				// Hit:
				range = std::hypotf( callback.m_point.x - sensorPt.x, callback.m_point.y - sensorPt.y );
				range += mrpt::random::randomGenerator.drawGaussian1D_normalized() * rangeStdNoise;
			}
			else
			{
				// Miss:
				range = maxRange;
			}
		} // end for (raycast scan)
	}	
	m_world->getTimeLogger().leave("LaserScanner.scan.2.polygons");

	// Summarize all scans in one single scan:
	// ----------------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.3.merge");

	mrpt::slam::CObservation2DRangeScan *lastScan = new mrpt::slam::CObservation2DRangeScan(m_scan_model);
	lastScan->timestamp = mrpt::system::now();
	
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
	m_world->getTimeLogger().leave("LaserScanner.scan.3.merge");

	{
		mrpt::synch::CCriticalSectionLocker csl(&m_last_scan_cs);
		m_last_scan = mrpt::slam::CObservation2DRangeScanPtr( lastScan );
		m_last_scan2gui = m_last_scan;
	}
	m_gui_uptodate = false;

}
