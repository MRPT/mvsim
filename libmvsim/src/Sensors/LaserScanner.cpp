/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/random.h>
#include <mvsim/Sensors/LaserScanner.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

int z_order_cnt = 0;

LaserScanner::LaserScanner(
	VehicleBase& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent),
	  m_z_order(++z_order_cnt),
	  m_rangeStdNoise(0.01),
	  m_angleStdNoise(mrpt::DEG2RAD(0.01)),
	  m_see_fixtures(true)
{
	this->loadConfigFrom(root);
}

LaserScanner::~LaserScanner() {}
void LaserScanner::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	m_gui_uptodate = false;

	// Attribs:
	TParameterDefinitions attribs;
	attribs["name"] = TParamEntry("%s", &this->m_name);

	parse_xmlnode_attribs(*root, attribs, "[LaserScanner]");

	// Other scalar params:
	int nRays = 181;
	double fov_deg = 180;

	TParameterDefinitions params;
	params["fov_degrees"] = TParamEntry("%lf", &fov_deg);
	params["nrays"] = TParamEntry("%i", &nRays);
	params["pose"] = TParamEntry("%pose2d_ptr3d", &m_scan_model.sensorPose);
	params["height"] = TParamEntry("%lf", &m_scan_model.sensorPose.z());
	params["range_std_noise"] = TParamEntry("%lf", &m_rangeStdNoise);
	params["angle_std_noise_deg"] = TParamEntry("%lf_deg", &m_angleStdNoise);
	params["sensor_period"] = TParamEntry("%lf", &this->m_sensor_period);
	params["bodies_visible"] = TParamEntry("%bool", &this->m_see_fixtures);

	m_scan_model.sensorPose.z() = 0.05;

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params);

	// Pass params to the scan2D obj:
	m_scan_model.aperture = mrpt::DEG2RAD(fov_deg);
	m_scan_model.resizeScan(nRays);

	// Assign a sensible default name/sensor label if none is provided:
	if (m_name.empty())
	{
		const size_t nextIdx = m_vehicle.getSensors().size() + 1;
		m_name = mrpt::format("laser%u", static_cast<unsigned int>(nextIdx));
	}
}

void LaserScanner::internalGuiUpdate(mrpt::opengl::COpenGLScene& scene)
{
	// 1st time?
	if (!m_gl_scan)
	{
		m_gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
		m_gl_scan->enableSurface(false);
		// m_gl_scan->setSurfaceColor(0.0f, 0.0f, 1.0f, 0.4f);
		m_gl_scan->setLocalRepresentativePoint({0, 0, 0.10f});
		scene.insert(m_gl_scan);
	}

	if (!m_gui_uptodate)
	{
		{
			std::lock_guard<std::mutex> csl(m_last_scan_cs);
			if (m_last_scan2gui)
			{
				m_gl_scan->setScan(*m_last_scan2gui);
				m_last_scan2gui.reset();
			}
		}
		m_gui_uptodate = true;
	}

	const double z_incrs = 10e-3;  // for m_z_order
	const double z_offset = 10e-2;
	const mrpt::poses::CPose2D& p = m_vehicle.getCPose2D();
	m_gl_scan->setPose(mrpt::poses::CPose3D(
		p.x(), p.y(), z_offset + z_incrs * m_z_order, p.phi(), 0.0, 0.0));
}

void LaserScanner::simul_pre_timestep(const TSimulContext& context) {}
// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void LaserScanner::simul_post_timestep(const TSimulContext& context)
{
	using mrpt::maps::COccupancyGridMap2D;
	using mrpt::obs::CObservation2DRangeScan;

	// Limit sensor rate:
	if (context.simul_time < m_sensor_last_timestamp + m_sensor_period) return;

	m_sensor_last_timestamp = context.simul_time;

	// Create an array of scans, each reflecting ranges to one kind of world
	// objects.
	// Finally, we'll take the shortest range in each direction:
	std::list<CObservation2DRangeScan> lstScans;

	const size_t nRays = m_scan_model.getScanSize();
	const double maxRange = m_scan_model.maxRange;

	// Get pose of the robot:
	const mrpt::poses::CPose2D& vehPose = m_vehicle.getCPose2D();

	// grid maps:
	// -------------
	m_world->getTimeLogger().enter("LaserScanner.scan.1.gridmap");

	const World::TListWorldElements& elements =
		m_world->getListOfWorldElements();

	for (const auto& element : elements)
	{
		// If not a grid map, ignore:
		const OccupancyGridMap* grid =
			dynamic_cast<const OccupancyGridMap*>(element);
		if (!grid) continue;
		const COccupancyGridMap2D& occGrid = grid->getOccGrid();

		// Create new scan:
		lstScans.emplace_back(m_scan_model);
		CObservation2DRangeScan& scan = lstScans.back();

		// Ray tracing over the gridmap:
		occGrid.laserScanSimulator(
			scan, vehPose, 0.5f, m_scan_model.getScanSize(), m_rangeStdNoise, 1,
			m_angleStdNoise);
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.1.gridmap");

	// ray trace on Box2D polygons:
	// ------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.2.polygons");
	{
		// Create new scan:
		lstScans.push_back(CObservation2DRangeScan(m_scan_model));
		CObservation2DRangeScan& scan = lstScans.back();

		// Do Box2D raycasting stuff:
		// ------------------------------
		// This callback finds the closest hit. Polygon 0 is filtered.
		class RayCastClosestCallback : public b2RayCastCallback
		{
		   public:
			RayCastClosestCallback() : m_see_fixtures(true), m_hit(false) {}
			float32 ReportFixture(
				b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal,
				float32 fraction) override
			{
				if (!m_see_fixtures ||
					fixture->GetUserData() == INVISIBLE_FIXTURE_USER_DATA)
				{
					// By returning -1, we instruct the calling code to ignore
					// this fixture and
					// continue the ray-cast to the next fixture.
					return -1.0f;
				}

				m_hit = true;
				m_point = point;
				m_normal = normal;
				// By returning the current fraction, we instruct the calling
				// code to clip the ray and
				// continue the ray-cast to the next fixture. WARNING: do not
				// assume that fixtures
				// are reported in order. However, by clipping, we can always
				// get the closest fixture.
				return fraction;
			}

			bool m_see_fixtures;
			bool m_hit;
			b2Vec2 m_point;
			b2Vec2 m_normal;
		};

		const mrpt::poses::CPose2D sensorPose =
			vehPose + mrpt::poses::CPose2D(scan.sensorPose);
		const b2Vec2 sensorPt = b2Vec2(sensorPose.x(), sensorPose.y());

		RayCastClosestCallback callback;
		callback.m_see_fixtures = m_see_fixtures;

		// Scan size:
		ASSERT_(nRays >= 2);
		scan.resizeScan(nRays);
		double A =
			sensorPose.phi() + (scan.rightToLeft ? -0.5 : +0.5) * scan.aperture;
		const double AA =
			(scan.rightToLeft ? 1.0 : -1.0) * (scan.aperture / (nRays - 1));

		auto& rnd = mrpt::random::getRandomGenerator();

		for (size_t i = 0; i < nRays; i++, A += AA)
		{
			const b2Vec2 endPt = b2Vec2(
				sensorPt.x + cos(A) * maxRange, sensorPt.y + sin(A) * maxRange);

			callback.m_hit = false;
			m_world->getBox2DWorld()->RayCast(&callback, sensorPt, endPt);
			scan.setScanRangeValidity(i, callback.m_hit);

			float range = scan.getScanRange(i);
			if (callback.m_hit)
			{
				// Hit:
				range = ::hypotf(
					callback.m_point.x - sensorPt.x,
					callback.m_point.y - sensorPt.y);
				range += rnd.drawGaussian1D_normalized() * m_rangeStdNoise;
			}
			else
			{
				// Miss:
				range = maxRange;
			}
			scan.setScanRange(i, range);
		}  // end for (raycast scan)
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.2.polygons");

	// Summarize all scans in one single scan:
	// ----------------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.3.merge");

	auto lastScan = CObservation2DRangeScan::Create(m_scan_model);

	lastScan->timestamp = mrpt::system::now();
	lastScan->sensorLabel = m_name;

	lastScan->resizeScanAndAssign(nRays, maxRange, false);

	for (const auto& scan : lstScans)
	{
		for (size_t i = 0; i < nRays; i++)
		{
			if (scan.getScanRangeValidity(i))
			{
				lastScan->setScanRange(
					i,
					std::min(lastScan->getScanRange(i), scan.getScanRange(i)));
				lastScan->setScanRangeValidity(i, true);
			}
		}
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.3.merge");

	{
		std::lock_guard<std::mutex> csl(m_last_scan_cs);
		m_last_scan = std::move(lastScan);
		m_last_scan2gui = m_last_scan;
	}

	m_world->onNewObservation(m_vehicle, m_last_scan.get());

	m_gui_uptodate = false;
}
