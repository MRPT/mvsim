/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/WorldElements/OccupancyGridMap.h>
#include <mv2dsim/World.h>
#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <rapidxml.hpp>

using namespace rapidxml;
using namespace mv2dsim;
using namespace mrpt::slam;
using namespace std;

OccupancyGridMap::OccupancyGridMap(World*parent,const rapidxml::xml_node<char> *root) :
	WorldElementBase(parent),
	m_gui_uptodate(false),
	m_show_grid_collision_points(true)
{
	loadConfigFrom(root);
}

OccupancyGridMap::~OccupancyGridMap()
{
}

void OccupancyGridMap::loadConfigFrom(const rapidxml::xml_node<char> *root)
{
	m_gui_uptodate=false;

	// <file>FILENAME.{png,gridmap}</file>
	xml_node<> *xml_file = root->first_node("file");
	if (!xml_file || !xml_file->value())
		throw std::runtime_error("Error: <file></file> XML entry not found inside gridmap node!");

	const string sFile = xml_file->value();
	const string sFileExt = mrpt::system::extractFileExtension(sFile, true /*ignore gz*/);

	// MRPT gridmaps format:
	if ( sFileExt=="gridmap")
	{
		mrpt::utils::CFileGZInputStream f(sFile);
		f >> m_grid;
	}
	else
	// Assume it's an image:
	{
		std::map<std::string,TParamEntry> other_params;
		double xcenterpixel=-1,ycenterpixel=-1;
		double resolution=0.10;

		other_params["resolution"] = TParamEntry("%lf", &resolution);
		other_params["centerpixel_x"] = TParamEntry("%lf", &xcenterpixel);
		other_params["centerpixel_y"] = TParamEntry("%lf", &ycenterpixel);

		xml_node<> *child = root->first_node();
		while (child)
		{
			parse_xmlnode_as_param(*child,other_params);
			child = child->next_sibling();
		}

		m_grid.loadFromBitmapFile(sFile,resolution,xcenterpixel,ycenterpixel);
	}

}

void OccupancyGridMap::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	using namespace mrpt::math;

	// 1st time call?? -> Create objects
	if (!m_gl_grid)
	{
		m_gl_grid = mrpt::opengl::CSetOfObjects::Create();
		scene.insert(m_gl_grid);
	}
	if (m_gl_obs_clouds.size()!=m_obstacles_for_each_veh.size())
	{
		m_gl_obs_clouds.resize( m_obstacles_for_each_veh.size() );
	}

	// 1st call OR gridmap changed?
	if (!m_gui_uptodate)
	{
		m_grid.getAs3DObject(m_gl_grid);
		m_gui_uptodate=true;
	}
	// Update obstacles:
	for (size_t i=0;i<m_gl_obs_clouds.size();i++)
	{
		// 1st usage?
		mrpt::opengl::CSetOfObjectsPtr &gl_objs = m_gl_obs_clouds[i];
		if (!gl_objs)
		{
			gl_objs=mrpt::opengl::CSetOfObjects::Create();
			MRPT_TODO("Add a name, and remove old ones in scene, etc.")
			scene.insert(gl_objs);
		}

		mrpt::opengl::CPointCloudPtr gl_pts = gl_objs->getByClass<mrpt::opengl::CPointCloud>();
		if (!gl_pts)
		{
			gl_pts = mrpt::opengl::CPointCloud::Create();
			gl_pts->setPointSize(4.0f);
			gl_pts->setColor(0,0,1);
			gl_objs->insert(gl_pts);
		}

		mrpt::slam::CSimplePointsMap ptsmap;
		ptsmap.insertionOptions.minDistBetweenLaserPoints = 0;
		ptsmap.insertObservationPtr( m_obstacles_for_each_veh[i].scan );
		gl_pts->setVisibility( m_show_grid_collision_points );
		if (m_show_grid_collision_points)
		{
			gl_pts->loadFromPointsMap(&ptsmap);
			gl_pts->setPose( mrpt::poses::CPose2D( m_obstacles_for_each_veh[i].pose ) );
		}
	}

}

void OccupancyGridMap::simul_pre_timestep(const TSimulContext &context)
{
	// For each vehicle, create a ground body with fixtures at each scanned point around the vehicle, so it can collide with the environment:
	const list<VehicleBase*> & lstVehs =  this->m_world->getListOfVehicle();

	// Upon first usage, reserve mem:
	m_obstacles_for_each_veh.resize( lstVehs.size() );

	size_t veh_idx=0;
	for (list<VehicleBase*>::const_iterator itVeh=lstVehs.begin();itVeh!=lstVehs.end();++itVeh, ++veh_idx)
	{
		// 1) Simulate scan to get obstacles around the vehicle:
		TInfoPerVeh &ipv = m_obstacles_for_each_veh[veh_idx];
		mrpt::slam::CObservation2DRangeScanPtr &scan = ipv.scan;
		// Upon first time, reserve mem:
		if (!scan) scan = mrpt::slam::CObservation2DRangeScan::Create();

		const float veh_max_obstacles_ranges = (*itVeh)->getMaxVehicleRadius() * 1.75f;
		const float occup_threshold = 0.5f;
		const size_t nRays = 120;

		const vec3 &pose = (*itVeh)->getPose();
		scan->aperture = 2.0*M_PI; // 360 field of view
		scan->maxRange = veh_max_obstacles_ranges;

		ipv.pose =  mrpt::poses::CPose2D(pose.vals[0],pose.vals[1], 0 /* angle=0, no need to rotate everything to latter rotate back again! */);

		m_grid.laserScanSimulator(
			*scan,
			ipv.pose,
			occup_threshold, nRays, 0.0f /*noise*/ );

		// Since we'll dilate obstacle points, let's give a bit more space as compensation:
		const float range_enlarge = 0.5f*m_grid.getResolution();
		for (size_t k=0;k<scan->scan.size();k++)
			scan->scan[k]+=range_enlarge;

		// 2) Create a Box2D "ground body" with square "fixtures" so the vehicle can collide with the occ. grid:
		b2World * b2world = m_world->getBox2DWorld();

		// Create Box2D objects upon first usage:
		if (!ipv.collide_body)
		{
			b2BodyDef bdef;
			ipv.collide_body = b2world->CreateBody(&bdef);
		}
		// Force move the body to the vehicle origins (to use obstacles in local coords):
		ipv.collide_body->SetTransform( b2Vec2(ipv.pose.x(),ipv.pose.y()), .0f);

		// Physical properties of each "occupied cell":
		const float occCellSemiWidth = m_grid.getResolution()*0.4f;
		b2PolygonShape sqrPoly;
		sqrPoly.SetAsBox(occCellSemiWidth,occCellSemiWidth);
 		sqrPoly.m_radius = 1e-3;  // The "skin" depth of the body
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &sqrPoly;
		fixtureDef.restitution = 0.05;
		fixtureDef.density = 0; // Fixed (inf. mass)
		fixtureDef.friction = 0.3f;

		// Create fixtures at their place (or disable it if no obstacle has been sensed):

		const mrpt::slam::CSinCosLookUpTableFor2DScans::TSinCosValues & sincos_tab = m_sincos_lut.getSinCosForScan(*scan);
		ipv.collide_fixtures.resize(nRays);
		for (size_t k=0;k<nRays;k++)
		{
			if (!ipv.collide_fixtures[k].fixture)
				ipv.collide_fixtures[k].fixture = ipv.collide_body->CreateFixture(&fixtureDef);

			if (!scan->validRange[k])
			{
				ipv.collide_fixtures[k].fixture->SetSensor(true); // Box2D's way of saying: don't collide with this!
			}
			else
			{
				ipv.collide_fixtures[k].fixture->SetSensor(false); // Box2D's way of saying: don't collide with this!

				b2PolygonShape *poly = dynamic_cast<b2PolygonShape*>( ipv.collide_fixtures[k].fixture->GetShape() );
				ASSERT_(poly!=NULL)

				const float lx = sincos_tab.ccos[k] * scan->scan[k];
				const float ly = sincos_tab.csin[k] * scan->scan[k];
				poly->SetAsBox(occCellSemiWidth,occCellSemiWidth, b2Vec2(lx,ly), .0f /*angle*/);
			}
		}

	}

}

void OccupancyGridMap::simul_post_timestep(const TSimulContext &context)
{

}
