/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mvsim/WorldElements/WorldElementBase.h>

namespace mvsim
{
class GroundGrid : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(GroundGrid)
   public:
	GroundGrid(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~GroundGrid();

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;
	void poses_mutex_lock() override {}
	void poses_mutex_unlock() override {}

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;

	bool m_is_floating;
	std::string m_float_center_at_vehicle_name;
	double m_x_min, m_x_max, m_y_min, m_y_max, m_interval;
	mrpt::img::TColor m_color;
	double m_line_width;

	mrpt::opengl::CGridPlaneXY::Ptr m_gl_groundgrid;
};
}  // namespace mvsim
