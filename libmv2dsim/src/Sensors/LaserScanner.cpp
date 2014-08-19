/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/Sensors/LaserScanner.h>
#include "xml_utils.h"

using namespace mv2dsim;
using namespace rapidxml;

LaserScanner::LaserScanner(VehicleBase&parent,const rapidxml::xml_node<char> *root) :
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
}

void LaserScanner::simul_pre_timestep(const TSimulContext &context)
{
}

void LaserScanner::simul_post_timestep(const TSimulContext &context)
{
}
