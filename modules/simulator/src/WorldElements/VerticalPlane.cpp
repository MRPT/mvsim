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
#include <mvsim/WorldElements/VerticalPlane.h>

#include <rapidxml.hpp>

#include "JointXMLnode.h"
#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

VerticalPlane::VerticalPlane(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	// Create opengl object: in this class, we'll store most state data directly
	// in the mrpt::opengl object.
	VerticalPlane::loadConfigFrom(root);
}

VerticalPlane::~VerticalPlane() {}

void VerticalPlane::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	ASSERT_(root);

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

	params["x0"] = TParamEntry("%f", &x0_);
	params["x1"] = TParamEntry("%f", &x1_);
	params["y0"] = TParamEntry("%f", &y0_);
	params["y1"] = TParamEntry("%f", &y1_);
	params["z"] = TParamEntry("%f", &z_);
	params["height"] = TParamEntry("%f", &height_);
	params["cull_face"] = TParamEntry("%s", &cull_faces_);

	params["texture"] = TParamEntry("%s", &textureFileName_);
	params["texture_size_x"] = TParamEntry("%lf", &textureSizeX_);
	params["texture_size_y"] = TParamEntry("%lf", &textureSizeY_);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// Create box2d fixtures, to enable collision detection:
	// --------------------------------------------------------
	b2World& world = *world_->getBox2DWorld();

	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;

	b2dBody_ = world.CreateBody(&bodyDef);

	// Define shape of block:
	// ------------------------------
	{
		const float thickness = 0.02f;

		const mrpt::math::TPoint2Df p0 = {x0_, y0_};
		const mrpt::math::TPoint2Df p1 = {x1_, y1_};
		ASSERT_(p0 != p1);
		const auto v01 = p1 - p0;
		const mrpt::math::TPoint2Df normal = {-v01.y, v01.x};
		const auto w = normal.unitarize() * thickness;

		// Convert shape into Box2D format:
		const size_t nPts = 4;
		std::vector<b2Vec2> pts;

		const auto p00 = p0 - w, p01 = p0 + w;
		const auto p10 = p1 - w, p11 = p1 + w;

		pts.emplace_back(p00.x, p00.y);
		pts.emplace_back(p01.x, p01.y);
		pts.emplace_back(p11.x, p11.y);
		pts.emplace_back(p10.x, p10.y);

		b2PolygonShape blockPoly;
		blockPoly.Set(&pts[0], nPts);

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &blockPoly;
		fixtureDef.restitution = restitution_;

		// Add the shape to the body.
		fixture_block_ = b2dBody_->CreateFixture(&fixtureDef);
	}

	// Init pos: (the vertices already have the global coordinates)
	b2dBody_->SetTransform(b2Vec2(0, 0), 0);
}

void VerticalPlane::internalGuiUpdate(
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
		const mrpt::math::TPoint3Df p0 = {x0_, y0_, z_};
		const mrpt::math::TPoint3Df p1 = {x1_, y1_, z_};

		ASSERT_(p0 != p1);
		const auto v01 = p1 - p0;

		const float L = v01.norm(), H = height_;

		const auto center = (p0 + p1) * 0.5f + mrpt::math::TPoint3Df(0, 0, 0.5f * H);

		gl_plane_ = mrpt::opengl::CTexturedPlane::Create();
		gl_plane_->setPlaneCorners(-0.5 * L, 0.5 * L, -0.5 * H, 0.5 * H);

		mrpt::math::TPose3D p;
		p.x = center.x;
		p.y = center.y;
		p.z = center.z;
		p.yaw = std::atan2(v01.y, v01.x);
		p.roll = mrpt::DEG2RAD(90.0);

		const auto pp = parent()->applyWorldRenderOffset(p);

		gl_plane_->setPose(pp);
		gl_plane_->setName("VerticalPlane_"s + getName());

#if MRPT_VERSION >= 0x270
		gl_plane_->enableLighting(enableShadows_);
#endif

		gl_plane_->setColor_u8(color_);

#if MRPT_VERSION >= 0x240
		gl_plane_->cullFaces(
			mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(cull_faces_));
#endif
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

		const mrpt::math::TPoint3Df p0 = {x0_, y0_, z_};
		const mrpt::math::TPoint3Df p1 = {x1_, y1_, z_};

		ASSERT_(p0 != p1);
		const auto v01 = p1 - p0;

		const float L = v01.norm(), H = height_;

		// Compute (U,V) texture coordinates:
		float u_min = 0;
		float v_min = 0;
		float u_max = L / textureSizeX_;
		float v_max = H / textureSizeY_;

		gl_plane_text_ = mrpt::opengl::CSetOfTexturedTriangles::Create();
		gl_plane_text_->setName("VerticalPlane_"s + getName());

		gl_plane_text_->enableLight(enableShadows_);

		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = {x0_, y0_, z_};
			t.vertices[1].xyzrgba.pt = {x1_, y1_, z_};
			t.vertices[2].xyzrgba.pt = {x1_, y1_, z_ + H};

			t.vertices[0].uv = {u_min, v_min};
			t.vertices[1].uv = {u_max, v_min};
			t.vertices[2].uv = {u_max, v_max};

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = {x0_, y0_, z_};
			t.vertices[1].xyzrgba.pt = {x1_, y1_, z_ + H};
			t.vertices[2].xyzrgba.pt = {x0_, y0_, z_ + H};

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

void VerticalPlane::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
}

void VerticalPlane::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
}
