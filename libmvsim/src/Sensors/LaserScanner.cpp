/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/Sensors/LaserScanner.h>
#include <mvsim/VehicleBase.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/random.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

int z_order_cnt = 0;

LaserScanner::LaserScanner(VehicleBase&parent,const rapidxml::xml_node<char> *root) :
	SensorBase(parent),
	m_z_order(++z_order_cnt),
	m_rangeStdNoise(0.01),
	m_angleStdNoise( mrpt::utils::DEG2RAD(0.01) ),
	m_see_fixtures(true)
{
	this->loadConfigFrom(root);
}

LaserScanner::~LaserScanner()
{
}

void LaserScanner::loadConfigFrom(const rapidxml::xml_node<char> *root)
{
	m_gui_uptodate=false;

	// Attribs:
	std::map<std::string,TParamEntry> attribs;
	attribs["name"] = TParamEntry("%s", &this->m_name);

	parse_xmlnode_attribs(*root, attribs,"[LaserScanner]");

	// Other scalar params:
	int nRays = 181;
	double fov_deg = 180;

	std::map<std::string,TParamEntry> params;
	params["fov_degrees"] = TParamEntry("%lf",&fov_deg);
	params["nrays"] = TParamEntry("%i",&nRays);
	params["pose"] = TParamEntry("%pose2d_ptr3d",&m_scan_model.sensorPose);
	params["range_std_noise"] = TParamEntry("%lf",&m_rangeStdNoise);
	params["angle_std_noise_deg"] = TParamEntry("%lf_deg",&m_angleStdNoise);
	params["sensor_period"] = TParamEntry("%lf", &this->m_sensor_period);
	params["bodies_visible"] = TParamEntry("%bool", &this->m_see_fixtures);


	// Parse XML params:
	parse_xmlnode_children_as_param(*root,params);

	// Pass params to the scan2D obj:
	m_scan_model.aperture = mrpt::utils::DEG2RAD(fov_deg);
#if MRPT_VERSION>=0x150
	m_scan_model.resizeScan(nRays);
#else
	m_scan_model.scan.resize(nRays);
	m_scan_model.validRange.resize(nRays);
#endif

	// Assign a sensible default name/sensor label if none is provided:
	if (m_name.empty()) {
		const size_t nextIdx = m_vehicle.getSensors().size()+1;
		m_name = mrpt::format("laser%u",static_cast<unsigned int>(nextIdx));
	}
}

void LaserScanner::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	// 1st time?
	if (!m_gl_scan)
	{
		m_gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
		m_gl_scan->setSurfaceColor(0.0f, 0.0f, 1.0f, 0.05f);
		SCENE_INSERT_Z_ORDER(scene,2, m_gl_scan);
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
	// Limit sensor rate:
	if (context.simul_time<m_sensor_last_timestamp+m_sensor_period)
		return;

	m_sensor_last_timestamp = context.simul_time;

	// Create an array of scans, each reflecting ranges to one kind of world objects.
	// Finally, we'll take the shortest range in each direction:
	std::list<CObservation2DRangeScan> lstScans;

	const size_t nRays = m_scan_model.scan.size();
	const double maxRange = m_scan_model.maxRange;

	// Get pose of the robot:
	const mrpt::poses::CPose2D &vehPose = m_vehicle.getCPose2D();

	// grid maps:
	// -------------
	m_world->getTimeLogger().enter("LaserScanner.scan.1.gridmap");

	const World::TListWorldElements &elements = m_world->getListOfWorldElements();
	for (World::TListWorldElements::const_iterator it=elements.begin();it!=elements.end();++it)
	{
		// If not a grid map, ignore:
		const OccupancyGridMap* grid = dynamic_cast<const OccupancyGridMap*>(*it);
		if (!grid) continue;
		const COccupancyGridMap2D &occGrid = grid->getOccGrid();

		// Create new scan:
		lstScans.push_back( CObservation2DRangeScan(m_scan_model) );
		CObservation2DRangeScan &scan = lstScans.back();

		// Ray tracing over the gridmap:
		occGrid.laserScanSimulator(scan,vehPose,0.5f, m_scan_model.scan.size(), m_rangeStdNoise, 1, m_angleStdNoise);
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.1.gridmap");

	// ray trace on Box2D polygons:
	// ------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.2.polygons");
	{
		// Create new scan:
		lstScans.push_back( CObservation2DRangeScan(m_scan_model) );
		CObservation2DRangeScan &scan = lstScans.back();

		// Do Box2D raycasting stuff:
		// ------------------------------
		// This callback finds the closest hit. Polygon 0 is filtered.
		class RayCastClosestCallback : public b2RayCastCallback
		{
		public:
			RayCastClosestCallback() : m_see_fixtures (true), m_hit(false)
			{
			}

			float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
			{
				if (!m_see_fixtures ||
				    fixture->GetUserData()==INVISIBLE_FIXTURE_USER_DATA)
				{
					// By returning -1, we instruct the calling code to ignore this fixture and
					// continue the ray-cast to the next fixture.
					return -1.0f;
				}

				m_hit = true;
				m_point = point;
				m_normal = normal;
				// By returning the current fraction, we instruct the calling code to clip the ray and
				// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
				// are reported in order. However, by clipping, we can always get the closest fixture.
				return fraction;
			}

			bool m_see_fixtures;
			bool m_hit;
			b2Vec2 m_point;
			b2Vec2 m_normal;
		};

		const mrpt::poses::CPose2D sensorPose = vehPose + mrpt::poses::CPose2D(scan.sensorPose);
		const b2Vec2 sensorPt = b2Vec2(sensorPose.x(),sensorPose.y());

		RayCastClosestCallback callback;
		callback.m_see_fixtures = m_see_fixtures;

		// Scan size:
		ASSERT_(nRays>=2)
#if MRPT_VERSION>=0x150
		scan.resizeScan(nRays);
#else
		scan.scan.resize(nRays);
		scan.validRange.resize(nRays);
#endif
		double  A = sensorPose.phi() + (scan.rightToLeft ? -0.5:+0.5) *scan.aperture;
		const double AA = (scan.rightToLeft ? 1.0:-1.0) * (scan.aperture / (nRays-1));

		for (size_t i=0;i<nRays;i++,A+=AA)
		{
			const b2Vec2 endPt = b2Vec2(sensorPt.x + cos(A)*maxRange,sensorPt.y + sin(A)*maxRange);

			callback.m_hit=false;
			m_world->getBox2DWorld()->RayCast(&callback, sensorPt, endPt);
#if MRPT_VERSION>=0x150
			scan.setScanRangeValidity(i,callback.m_hit);
#else
			scan.validRange[i] = callback.m_hit ? 1:0;
#endif

			float range = scan.scan[i];
			if (callback.m_hit)
			{
				// Hit:
				range = ::hypotf( callback.m_point.x - sensorPt.x, callback.m_point.y - sensorPt.y );
				range += mrpt::random::randomGenerator.drawGaussian1D_normalized() * m_rangeStdNoise;
			}
			else
			{
				// Miss:
				range = maxRange;
			}
#if MRPT_VERSION>=0x150
			scan.setScanRange(i, range);
#else
			scan.scan[i] = range;
#endif
		} // end for (raycast scan)
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.2.polygons");

	// Summarize all scans in one single scan:
	// ----------------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.3.merge");

	CObservation2DRangeScan *lastScan = new CObservation2DRangeScan(m_scan_model);
	lastScan->timestamp = mrpt::system::now();
	lastScan->sensorLabel = m_name;

#if MRPT_VERSION>=0x150
	lastScan->resizeScanAndAssign(nRays, maxRange, false);
#else
	lastScan->scan.assign(nRays,maxRange);
	lastScan->validRange.assign(nRays, 0);
#endif

	for (std::list<CObservation2DRangeScan>::const_iterator it=lstScans.begin();it!=lstScans.end();++it)
	{
		ASSERT_(it->scan.size()==nRays && it->validRange.size()==nRays)

		for (size_t i=0;i<nRays;i++)
		{
			if (it->validRange[i])
			{
#if MRPT_VERSION>=0x150
				lastScan->setScanRange(i, std::min(lastScan->scan[i], it->scan[i]) );
				lastScan->setScanRangeValidity(i, true );
#else
				lastScan->scan[i] = std::min(lastScan->scan[i], it->scan[i]);
				lastScan->validRange[i] = 1; // valid
#endif
			}
		}
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.3.merge");

	{
		mrpt::synch::CCriticalSectionLocker csl(&m_last_scan_cs);
		m_last_scan = CObservation2DRangeScanPtr( lastScan );
		m_last_scan2gui = m_last_scan;
	}

	m_world->onNewObservation(m_vehicle, m_last_scan.pointer() );

	m_gui_uptodate = false;

}
