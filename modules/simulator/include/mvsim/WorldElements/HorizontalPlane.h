/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTexturedTriangles.h>
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
	// ------- Interface with "World" ------
	void simul_pre_timestep(const TSimulContext& context) override;
	void simul_post_timestep(const TSimulContext& context) override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	float x_min_ = -10, x_max_ = 10, y_min_ = -10, y_max_ = 10;
	mrpt::img::TColor color_ = {0xa0, 0xa0, 0xa0, 0xff};
	bool enableShadows_ = true;

	/** If defined, it overrides the plain color in color_ */
	std::string textureFileName_;
	double textureSizeX_ = 1.0;
	double textureSizeY_ = 1.0;

	float z_ = .0f;
	std::string cull_faces_ = "NONE";

	mrpt::opengl::CTexturedPlane::Ptr gl_plane_;
	mrpt::opengl::CSetOfTexturedTriangles::Ptr gl_plane_text_;
	mrpt::opengl::CSetOfObjects::Ptr glGroup_;
};
}  // namespace mvsim
