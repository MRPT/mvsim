/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/HorizontalPlane.h>

#include <rapidxml.hpp>

#include "JointXMLnode.h"
#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

HorizontalPlane::HorizontalPlane(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	// Create opengl object: in this class, we'll store most state data directly
	// in the mrpt::opengl object.
	HorizontalPlane::loadConfigFrom(root);
}

HorizontalPlane::~HorizontalPlane() {}

void HorizontalPlane::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	if (!root) return;	// Assume defaults

	// Common setup for simulable objects:
	// -----------------------------------------------------------
	{
		ParseSimulableParams p;
		p.init_pose_mandatory = false;

		JointXMLnode<> jnode;
		jnode.add(root);
		parseSimulable(jnode, p);
	}

	TParameterDefinitions params;
	params["color"] = TParamEntry("%color", &color_);
	params["enable_shadows"] = TParamEntry("%bool", &enableShadows_);

	params["x_min"] = TParamEntry("%f", &x_min_);
	params["x_max"] = TParamEntry("%f", &x_max_);
	params["y_min"] = TParamEntry("%f", &y_min_);
	params["y_max"] = TParamEntry("%f", &y_max_);
	params["z"] = TParamEntry("%f", &z_);
	params["cull_face"] = TParamEntry("%s", &cull_faces_);

	params["texture"] = TParamEntry("%s", &textureFileName_);
	params["texture_size_x"] = TParamEntry("%lf", &textureSizeX_);
	params["texture_size_y"] = TParamEntry("%lf", &textureSizeY_);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());
}

void HorizontalPlane::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;
	using namespace std::string_literals;

	if (!glGroup_) glGroup_ = mrpt::opengl::CSetOfObjects::Create();

	// 1st call? (w/o texture)
	if (!gl_plane_ && textureFileName_.empty() && viz && physical)
	{
		gl_plane_ = mrpt::opengl::CTexturedPlane::Create();
		gl_plane_->setPlaneCorners(x_min_, x_max_, y_min_, y_max_);
		gl_plane_->setLocation(0, 0, z_);
		gl_plane_->setName("HorizontalPlane_"s + getName());

		gl_plane_->enableLighting(enableShadows_);
		gl_plane_->setColor_u8(color_);
		gl_plane_->cullFaces(
			mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(cull_faces_));
		glGroup_->insert(gl_plane_);
		viz->get().insert(glGroup_);
		physical->get().insert(glGroup_);
	}
	// 1st call? (with texture)
	if (!gl_plane_text_ && !textureFileName_.empty() && viz && physical)
	{
		const std::string localFileName = world_->xmlPathToActualPath(textureFileName_);
		ASSERT_FILE_EXISTS_(localFileName);

		mrpt::img::CImage texture;
		bool textureReadOk = texture.loadFromFile(localFileName);
		ASSERT_(textureReadOk);

		// Compute (U,V) texture coordinates:
		float u_min = 0;
		float v_min = 0;
		float u_max = (x_max_ - x_min_) / textureSizeX_;
		float v_max = (y_max_ - y_min_) / textureSizeY_;

		gl_plane_text_ = mrpt::opengl::CSetOfTexturedTriangles::Create();
		gl_plane_text_->setName("HorizontalPlane_"s + getName());

		gl_plane_text_->enableLight(enableShadows_);

		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = {x_min_, y_min_, z_};
			t.vertices[1].xyzrgba.pt = {x_max_, y_min_, z_};
			t.vertices[2].xyzrgba.pt = {x_max_, y_max_, z_};

			t.vertices[0].uv = {u_min, v_min};
			t.vertices[1].uv = {u_max, v_min};
			t.vertices[2].uv = {u_max, v_max};

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = {x_min_, y_min_, z_};
			t.vertices[1].xyzrgba.pt = {x_max_, y_max_, z_};
			t.vertices[2].xyzrgba.pt = {x_min_, y_max_, z_};

			t.vertices[0].uv = {u_min, v_min};
			t.vertices[1].uv = {u_max, v_max};
			t.vertices[2].uv = {u_min, v_max};

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}

		gl_plane_text_->assignImage(texture);

#if MRPT_VERSION >= 0x240
		gl_plane_text_->cullFaces(
			mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(cull_faces_));
#endif

		glGroup_->insert(gl_plane_text_);
		viz->get().insert(glGroup_);
		physical->get().insert(glGroup_);
	}

	// Update them:
	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we
	// don't need/can't acquire it again:
	const auto objectPose = viz.has_value() ? getPose() : getPoseNoLock();

	glGroup_->setPose(objectPose);
}

void HorizontalPlane::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
}

void HorizontalPlane::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
}

std::optional<float> HorizontalPlane::getElevationAt(const mrpt::math::TPoint2Df& worldXY) const
{
	const auto& myPose = getCPose3D();

	const auto localPt =
		getCPose3D().inverseComposePoint(mrpt::math::TPoint3D(worldXY.x, worldXY.y, .0));

	if (localPt.x < x_min_ || localPt.x > x_max_ || localPt.y < y_min_ || localPt.y > y_max_)
	{
		// Out of the plane:
		return {};
	}

	auto p = myPose + mrpt::poses::CPose3D::FromTranslation(0, 0, z_);
	return p.z();
}
