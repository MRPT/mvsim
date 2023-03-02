/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
/** A vertical plane, visible from one or both sides (see "cull_faces"),
 *  conveniently defined by starting and end points (x0,y0)-(x1,y1).
 *
 */
class VerticalPlane : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(VerticalPlane)
   public:
	VerticalPlane(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~VerticalPlane();

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;
	// ------- Interface with "World" ------
	void simul_pre_timestep(const TSimulContext& context) override;
	void simul_post_timestep(const TSimulContext& context) override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		bool childrenOnly) override;

	float x0_ = -10, x1_ = -10, y0_ = -10, y1_ = 10;
	mrpt::img::TColor color_ = {0xa0, 0xa0, 0xa0, 0xff};
	bool enableShadows_ = true;

	/** If defined, it overrides the plain color in color_ */
	std::string textureFileName_;
	double textureSizeX_ = 1.0;
	double textureSizeY_ = 1.0;

	float z_ = .0f, height_ = 3.0f;
	std::string cull_faces_ = "NONE";

	mrpt::opengl::CTexturedPlane::Ptr gl_plane_;
	mrpt::opengl::CSetOfTexturedTriangles::Ptr gl_plane_text_;
	mrpt::opengl::CSetOfObjects::Ptr glGroup_;

	b2Body* b2dBody_ = nullptr;
	double restitution_ = 0.01;	 //!< Default: 0.01
	b2Fixture* fixture_block_ = nullptr;
};
}  // namespace mvsim
