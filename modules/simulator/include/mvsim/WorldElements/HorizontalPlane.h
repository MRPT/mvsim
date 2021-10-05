/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mvsim/WorldElements/WorldElementBase.h>

namespace mvsim
{
class HorizontalPlane : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(HorizontalPlane)
   public:
	HorizontalPlane(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~HorizontalPlane();

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;
	void poses_mutex_lock() override {}
	void poses_mutex_unlock() override {}

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;

	double m_x_min = -10, m_x_max = 10, m_y_min = -10, m_y_max = 10;
	mrpt::img::TColor m_color = {0xa0, 0xa0, 0xa0, 0xff};
	double m_z = .0;
	std::string m_cull_faces = "NONE";

	mrpt::opengl::CTexturedPlane::Ptr m_gl_plane;
};
}  // namespace mvsim
