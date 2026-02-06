/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/VerticalPlane.h>

#include <iostream>
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

bool VerticalPlane::Opening::overlaps(float pos_start, float pos_end) const
{
	const float opening_start = position - width * 0.5f;
	const float opening_end = position + width * 0.5f;

	return !(opening_end < pos_start || opening_start > pos_end);
}

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
	params["thickness"] = TParamEntry("%f", &thickness_);
	params["cull_face"] = TParamEntry("%s", &cull_faces_);

	params["texture"] = TParamEntry("%s", &textureFileName_);
	params["texture_size_x"] = TParamEntry("%lf", &textureSizeX_);
	params["texture_size_y"] = TParamEntry("%lf", &textureSizeY_);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// Parse openings (doors and windows):
	// --------------------------------------------------------
	openings_.clear();

	for (xml_node<>* node = root->first_node(); node; node = node->next_sibling())
	{
		const std::string tagName = node->name();

		if (tagName == "door" || tagName == "window")
		{
			Opening opening;
			opening.type = (tagName == "door") ? Opening::Type::DOOR : Opening::Type::WINDOW;

			// Default values
			opening.position = 0.5f;  // Middle of wall
			opening.width = 1.0f;
			opening.z_min = 0.0f;
			opening.z_max = 2.1f;

			// Parse attributes
			for (xml_attribute<>* attr = node->first_attribute(); attr;
				 attr = attr->next_attribute())
			{
				const std::string attrName = attr->name();
				const std::string attrValue = attr->value();

				if (attrName == "position")
				{
					opening.position = std::stof(attrValue);
				}
				else if (attrName == "width")
				{
					opening.width = std::stof(attrValue);
				}
				else if (attrName == "z_min" || attrName == "z_start")
				{
					opening.z_min = std::stof(attrValue);
				}
				else if (attrName == "z_max" || attrName == "z_end")
				{
					opening.z_max = std::stof(attrValue);
				}
				else if (attrName == "name")
				{
					opening.name = attrValue;
				}
			}

			// Parse child elements
			TParameterDefinitions opening_params;
			opening_params["position"] = TParamEntry("%f", &opening.position);
			opening_params["width"] = TParamEntry("%f", &opening.width);
			opening_params["z_min"] = TParamEntry("%f", &opening.z_min);
			opening_params["z_max"] = TParamEntry("%f", &opening.z_max);
			opening_params["name"] = TParamEntry("%s", &opening.name);

			parse_xmlnode_children_as_param(
				*node, opening_params, world_->user_defined_variables());

			// Validate and add
			if (opening.position < 0.0f || opening.position > 1.0f)
			{
				std::cerr << "[VerticalPlane] Warning: Opening position must be in [0,1], got "
						  << opening.position << ". Clamping." << std::endl;
				opening.position = std::max(0.0f, std::min(1.0f, opening.position));
			}

			if (opening.width <= 0.0f)
			{
				std::cerr << "[VerticalPlane] Warning: Opening width must be positive, got "
						  << opening.width << ". Ignoring opening." << std::endl;
				continue;
			}

			openings_.push_back(opening);
		}
	}

	// Create box2d fixtures:
	createPhysicsBodies();
}

void VerticalPlane::createPhysicsBodies()
{
	// Clean up old fixtures if they exist
	if (b2dBody_)
	{
		for (auto* fixture : fixtures_)
		{
			if (fixture)
			{
				b2dBody_->DestroyFixture(fixture);
			}
		}
		fixtures_.clear();
		world_->getBox2DWorld()->DestroyBody(b2dBody_);
		b2dBody_ = nullptr;
	}

	b2World& world = *world_->getBox2DWorld();

	// Define the static body
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	b2dBody_ = world.CreateBody(&bodyDef);

	const mrpt::math::TPoint2Df p0 = {x0_, y0_};
	const mrpt::math::TPoint2Df p1 = {x1_, y1_};
	ASSERT_(p0 != p1);

	const auto v01 = p1 - p0;
	const float wall_length = v01.norm();
	const mrpt::math::TPoint2Df dir = v01.unitarize();
	const mrpt::math::TPoint2Df normal = {-dir.y, dir.x};

	// Calculate inner and outer wall surfaces based on thickness
	const auto w_inner = normal * (thickness_ * 0.5f);
	const auto w_outer = normal * (-thickness_ * 0.5f);

	// Collect all door openings (windows still have collision)
	std::vector<std::pair<float, float>> door_ranges;  // (start_pos, end_pos) in meters

	for (const auto& opening : openings_)
	{
		if (opening.type == Opening::Type::DOOR)
		{
			const float pos_meters = opening.position * wall_length;
			const float half_width = opening.width * 0.5f;
			door_ranges.emplace_back(pos_meters - half_width, pos_meters + half_width);
		}
	}

	// Sort door ranges by start position
	std::sort(door_ranges.begin(), door_ranges.end());

	// Merge overlapping door ranges
	std::vector<std::pair<float, float>> merged_doors;
	for (const auto& range : door_ranges)
	{
		if (merged_doors.empty() || merged_doors.back().second < range.first)
		{
			merged_doors.push_back(range);
		}
		else
		{
			merged_doors.back().second = std::max(merged_doors.back().second, range.second);
		}
	}

	// Create wall segments between doors
	float current_pos = 0.0f;

	for (const auto& door : merged_doors)
	{
		if (current_pos < door.first)
		{
			// Create segment before this door
			const float seg_start = current_pos;
			const float seg_end = door.first;

			const auto seg_p0 = p0 + dir * seg_start;
			const auto seg_p1 = p0 + dir * seg_end;

			// Create a rectangular box with proper thickness
			const auto seg_p00 = seg_p0 + w_inner;
			const auto seg_p01 = seg_p0 + w_outer;
			const auto seg_p10 = seg_p1 + w_inner;
			const auto seg_p11 = seg_p1 + w_outer;

			std::vector<b2Vec2> pts;
			pts.emplace_back(seg_p00.x, seg_p00.y);
			pts.emplace_back(seg_p01.x, seg_p01.y);
			pts.emplace_back(seg_p11.x, seg_p11.y);
			pts.emplace_back(seg_p10.x, seg_p10.y);

			b2PolygonShape blockPoly;
			blockPoly.Set(&pts[0], 4);

			b2FixtureDef fixtureDef;
			fixtureDef.shape = &blockPoly;
			fixtureDef.restitution = restitution_;

			fixtures_.push_back(b2dBody_->CreateFixture(&fixtureDef));
		}

		current_pos = door.second;
	}

	// Create final segment after last door
	if (current_pos < wall_length)
	{
		const float seg_start = current_pos;
		const float seg_end = wall_length;

		const auto seg_p0 = p0 + dir * seg_start;
		const auto seg_p1 = p0 + dir * seg_end;

		const auto seg_p00 = seg_p0 + w_inner;
		const auto seg_p01 = seg_p0 + w_outer;
		const auto seg_p10 = seg_p1 + w_inner;
		const auto seg_p11 = seg_p1 + w_outer;

		std::vector<b2Vec2> pts;
		pts.emplace_back(seg_p00.x, seg_p00.y);
		pts.emplace_back(seg_p01.x, seg_p01.y);
		pts.emplace_back(seg_p11.x, seg_p11.y);
		pts.emplace_back(seg_p10.x, seg_p10.y);

		b2PolygonShape blockPoly;
		blockPoly.Set(&pts[0], 4);

		b2FixtureDef fixtureDef;
		fixtureDef.shape = &blockPoly;
		fixtureDef.restitution = restitution_;

		fixtures_.push_back(b2dBody_->CreateFixture(&fixtureDef));
	}

	// If no segments were created (entire wall is doors), create a dummy fixture
	if (fixtures_.empty())
	{
		// This shouldn't happen in practice, but handle it gracefully
		std::cerr << "[VerticalPlane] Warning: VerticalPlane has no collision geometry (all doors)."
				  << std::endl;
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

	// Create visual representation on first call
	if (!gl_plane_ && !gl_plane_text_ && viz && physical)
	{
		createVisualRepresentation(viz, physical);
	}

	// Update them:
	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we
	// don't need/can't acquire it again:
	const auto objectPoseOrg = viz.has_value() ? getPose() : getPoseNoLock();
	const auto objectPose = parent()->applyWorldRenderOffset(objectPoseOrg);

	glGroup_->setPose(objectPose);
}

void VerticalPlane::createVisualRepresentation(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical)
{
	using namespace mrpt::math;
	using namespace std::string_literals;

	const mrpt::math::TPoint2Df p0 = {x0_, y0_};
	const mrpt::math::TPoint2Df p1 = {x1_, y1_};

	ASSERT_(p0 != p1);
	const auto v01 = p1 - p0;
	const float wall_length = v01.norm();
	const auto dir = v01.unitarize();
	const mrpt::math::TPoint2Df normal = {-dir.y, dir.x};

	// Calculate inner and outer wall surfaces
	const auto w_inner = normal * (thickness_ * 0.5f);
	const auto w_outer = normal * (-thickness_ * 0.5f);

	// Load texture if specified
	mrpt::img::CImage texture;
	bool has_texture = false;

	if (!textureFileName_.empty())
	{
		const std::string localFileName = world_->xmlPathToActualPath(textureFileName_);
		ASSERT_FILE_EXISTS_(localFileName);
		has_texture = texture.loadFromFile(localFileName);
		ASSERT_(has_texture);
	}

	gl_plane_text_ = mrpt::opengl::CSetOfTexturedTriangles::Create();
	gl_plane_text_->setName("VerticalPlane_"s + getName());
	gl_plane_text_->enableLight(enableShadows_);

	// Render both front and back faces
	renderWallFace(p0, p1, w_inner, true, has_texture ? &texture : nullptr);
	renderWallFace(p0, p1, w_outer, false, has_texture ? &texture : nullptr);

	// Render edges (top, bottom, and sides) if wall has visible thickness
	if (thickness_ > 0.05f)	 // Only render edges for walls thicker than 5cm
	{
		renderWallEdges(p0, p1, w_inner, w_outer, has_texture ? &texture : nullptr);

		// Render reveals around door and window openings
		renderOpeningReveals(
			p0, p1, dir, w_inner, w_outer, wall_length, has_texture ? &texture : nullptr);
	}

	if (has_texture)
	{
		gl_plane_text_->assignImage(texture);
	}

	gl_plane_text_->cullFaces(
		mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(cull_faces_));

	glGroup_->insert(gl_plane_text_);
	viz->get().insert(glGroup_);
	physical->get().insert(glGroup_);
}

void VerticalPlane::renderWallFace(
	const mrpt::math::TPoint2Df& p0, const mrpt::math::TPoint2Df& p1,
	const mrpt::math::TPoint2Df& normal_offset, bool is_front_face,
	const mrpt::img::CImage* texture)
{
	using namespace mrpt::math;

	const auto v01 = p1 - p0;
	const float wall_length = v01.norm();
	const auto dir = v01.unitarize();

	// Build list of wall segments to render
	struct WallSegment
	{
		float pos_start;  // Position along wall (meters)
		float pos_end;
		float z_start;
		float z_end;
	};

	std::vector<WallSegment> segments;

	// Start with full wall
	segments.push_back({0.0f, wall_length, z_, z_ + height_});

	// Subtract each opening
	for (const auto& opening : openings_)
	{
		const float opening_pos = opening.position * wall_length;
		const float half_width = opening.width * 0.5f;
		const float opening_start = opening_pos - half_width;
		const float opening_end = opening_pos + half_width;

		std::vector<WallSegment> new_segments;

		for (const auto& seg : segments)
		{
			// Check if opening intersects this segment vertically
			if (opening.z_max <= seg.z_start || opening.z_min >= seg.z_end)
			{
				new_segments.push_back(seg);
				continue;
			}

			// Check horizontal intersection
			if (opening_end <= seg.pos_start || opening_start >= seg.pos_end)
			{
				new_segments.push_back(seg);
				continue;
			}

			// Opening intersects - split segment

			// Left segment
			if (seg.pos_start < opening_start)
			{
				new_segments.push_back(
					{seg.pos_start, std::min(opening_start, seg.pos_end), seg.z_start, seg.z_end});
			}

			// Right segment
			if (seg.pos_end > opening_end)
			{
				new_segments.push_back(
					{std::max(opening_end, seg.pos_start), seg.pos_end, seg.z_start, seg.z_end});
			}

			// Bottom segment
			if (seg.z_start < opening.z_min)
			{
				new_segments.push_back(
					{std::max(seg.pos_start, opening_start), std::min(seg.pos_end, opening_end),
					 seg.z_start, std::min(opening.z_min, seg.z_end)});
			}

			// Top segment
			if (seg.z_end > opening.z_max)
			{
				new_segments.push_back(
					{std::max(seg.pos_start, opening_start), std::min(seg.pos_end, opening_end),
					 std::max(opening.z_max, seg.z_start), seg.z_end});
			}
		}

		segments = new_segments;
	}

	// Create visual geometry for each segment
	for (const auto& seg : segments)
	{
		const auto seg_p0_2d = p0 + TPoint2Df(dir.x, dir.y) * seg.pos_start + normal_offset;
		const auto seg_p1_2d = p0 + TPoint2Df(dir.x, dir.y) * seg.pos_end + normal_offset;

		if (texture)
		{
			// Compute texture coordinates
			const float u_min = seg.pos_start / textureSizeX_;
			const float u_max = seg.pos_end / textureSizeX_;
			const float v_min = seg.z_start / textureSizeY_;
			const float v_max = seg.z_end / textureSizeY_;

			// Triangle 1
			{
				mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
				if (is_front_face)
				{
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_start};
					t.vertices[2].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};

					t.vertices[0].uv = {u_min, v_min};
					t.vertices[1].uv = {u_max, v_min};
					t.vertices[2].uv = {u_max, v_max};
				}
				else
				{
					// Reverse winding for back face
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};
					t.vertices[2].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_start};

					// UV coordinates must match the new vertex order
					t.vertices[0].uv = {u_min, v_min};	// p0, z_start
					t.vertices[1].uv = {u_max, v_max};	// p1, z_end
					t.vertices[2].uv = {u_max, v_min};	// p1, z_start
				}

				t.computeNormals();
				gl_plane_text_->insertTriangle(t);
			}

			// Triangle 2
			{
				mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
				if (is_front_face)
				{
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};
					t.vertices[2].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_end};

					t.vertices[0].uv = {u_min, v_min};
					t.vertices[1].uv = {u_max, v_max};
					t.vertices[2].uv = {u_min, v_max};
				}
				else
				{
					// Reverse winding for back face
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_end};
					t.vertices[2].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};

					// UV coordinates must match the new vertex order
					t.vertices[0].uv = {u_min, v_min};	// p0, z_start
					t.vertices[1].uv = {u_min, v_max};	// p0, z_end
					t.vertices[2].uv = {u_max, v_max};	// p1, z_end
				}

				t.computeNormals();
				gl_plane_text_->insertTriangle(t);
			}
		}
		else
		{
			// Plain color
			// Triangle 1
			{
				mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
				if (is_front_face)
				{
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_start};
					t.vertices[2].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};
				}
				else
				{
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};
					t.vertices[2].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_start};
				}

				t.vertices[0].setColor(color_);
				t.vertices[1].setColor(color_);
				t.vertices[2].setColor(color_);

				t.computeNormals();
				gl_plane_text_->insertTriangle(t);
			}

			// Triangle 2
			{
				mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
				if (is_front_face)
				{
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};
					t.vertices[2].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_end};
				}
				else
				{
					t.vertices[0].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_start};
					t.vertices[1].xyzrgba.pt = {seg_p0_2d.x, seg_p0_2d.y, seg.z_end};
					t.vertices[2].xyzrgba.pt = {seg_p1_2d.x, seg_p1_2d.y, seg.z_end};
				}

				t.vertices[0].setColor(color_);
				t.vertices[1].setColor(color_);
				t.vertices[2].setColor(color_);

				t.computeNormals();
				gl_plane_text_->insertTriangle(t);
			}
		}
	}
}

void VerticalPlane::renderWallEdges(
	const mrpt::math::TPoint2Df& p0, const mrpt::math::TPoint2Df& p1,
	const mrpt::math::TPoint2Df& w_inner, const mrpt::math::TPoint2Df& w_outer,
	const mrpt::img::CImage* texture)
{
	using namespace mrpt::math;

	const auto v01 = p1 - p0;
	const float wall_length = v01.norm();

	// Points at corners of the wall
	const auto p0_inner = p0 + w_inner;
	const auto p0_outer = p0 + w_outer;
	const auto p1_inner = p1 + w_inner;
	const auto p1_outer = p1 + w_outer;

	// Helper lambda to add a quad (2 triangles)
	auto addQuad = [this, texture](
					   const TPoint3Df& v0, const TPoint3Df& v1, const TPoint3Df& v2,
					   const TPoint3Df& v3, float u0, float v0_uv, float u1, float v1_uv)
	{
		// Triangle 1
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = v0;
			t.vertices[1].xyzrgba.pt = v1;
			t.vertices[2].xyzrgba.pt = v2;

			if (texture)
			{
				t.vertices[0].uv = {u0, v0_uv};
				t.vertices[1].uv = {u1, v0_uv};
				t.vertices[2].uv = {u1, v1_uv};
			}
			else
			{
				t.vertices[0].setColor(color_);
				t.vertices[1].setColor(color_);
				t.vertices[2].setColor(color_);
			}

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}

		// Triangle 2
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = v0;
			t.vertices[1].xyzrgba.pt = v2;
			t.vertices[2].xyzrgba.pt = v3;

			if (texture)
			{
				t.vertices[0].uv = {u0, v0_uv};
				t.vertices[1].uv = {u1, v1_uv};
				t.vertices[2].uv = {u0, v1_uv};
			}
			else
			{
				t.vertices[0].setColor(color_);
				t.vertices[1].setColor(color_);
				t.vertices[2].setColor(color_);
			}

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}
	};

	// Top edge (horizontal surface at top of wall)
	addQuad(
		{p0_inner.x, p0_inner.y, z_ + height_}, {p1_inner.x, p1_inner.y, z_ + height_},
		{p1_outer.x, p1_outer.y, z_ + height_}, {p0_outer.x, p0_outer.y, z_ + height_}, 0.0f, 0.0f,
		wall_length / textureSizeX_, thickness_ / textureSizeY_);

	// Bottom edge (horizontal surface at bottom of wall)
	addQuad(
		{p0_outer.x, p0_outer.y, z_}, {p1_outer.x, p1_outer.y, z_}, {p1_inner.x, p1_inner.y, z_},
		{p0_inner.x, p0_inner.y, z_}, 0.0f, 0.0f, wall_length / textureSizeX_,
		thickness_ / textureSizeY_);

	// Left edge (vertical surface at start of wall)
	addQuad(
		{p0_outer.x, p0_outer.y, z_}, {p0_inner.x, p0_inner.y, z_},
		{p0_inner.x, p0_inner.y, z_ + height_}, {p0_outer.x, p0_outer.y, z_ + height_}, 0.0f, 0.0f,
		thickness_ / textureSizeX_, height_ / textureSizeY_);

	// Right edge (vertical surface at end of wall)
	addQuad(
		{p1_inner.x, p1_inner.y, z_}, {p1_outer.x, p1_outer.y, z_},
		{p1_outer.x, p1_outer.y, z_ + height_}, {p1_inner.x, p1_inner.y, z_ + height_}, 0.0f, 0.0f,
		thickness_ / textureSizeX_, height_ / textureSizeY_);
}

void VerticalPlane::renderOpeningReveals(
	const mrpt::math::TPoint2Df& p0, [[maybe_unused]] const mrpt::math::TPoint2Df& p1,
	const mrpt::math::TPoint2Df& dir, const mrpt::math::TPoint2Df& w_inner,
	const mrpt::math::TPoint2Df& w_outer, float wall_length, const mrpt::img::CImage* texture)
{
	using namespace mrpt::math;

	// Helper lambda to add a quad (2 triangles)
	auto addQuad = [this, texture](
					   const TPoint3Df& v0, const TPoint3Df& v1, const TPoint3Df& v2,
					   const TPoint3Df& v3, float u0, float v0_uv, float u1, float v1_uv)
	{
		// Triangle 1
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = v0;
			t.vertices[1].xyzrgba.pt = v1;
			t.vertices[2].xyzrgba.pt = v2;

			if (texture)
			{
				t.vertices[0].uv = {u0, v0_uv};
				t.vertices[1].uv = {u1, v0_uv};
				t.vertices[2].uv = {u1, v1_uv};
			}
			else
			{
				t.vertices[0].setColor(color_);
				t.vertices[1].setColor(color_);
				t.vertices[2].setColor(color_);
			}

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}

		// Triangle 2
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = v0;
			t.vertices[1].xyzrgba.pt = v2;
			t.vertices[2].xyzrgba.pt = v3;

			if (texture)
			{
				t.vertices[0].uv = {u0, v0_uv};
				t.vertices[1].uv = {u1, v1_uv};
				t.vertices[2].uv = {u0, v1_uv};
			}
			else
			{
				t.vertices[0].setColor(color_);
				t.vertices[1].setColor(color_);
				t.vertices[2].setColor(color_);
			}

			t.computeNormals();
			gl_plane_text_->insertTriangle(t);
		}
	};

	// Render reveals for each opening
	for (const auto& opening : openings_)
	{
		const float opening_pos = opening.position * wall_length;
		const float half_width = opening.width * 0.5f;
		const float opening_start = opening_pos - half_width;
		const float opening_end = opening_pos + half_width;

		// Calculate the 4 corner positions of the opening
		const auto opening_p0 = p0 + dir * opening_start;  // Left edge
		const auto opening_p1 = p0 + dir * opening_end;	 // Right edge

		const auto opening_p0_inner = opening_p0 + w_inner;
		const auto opening_p0_outer = opening_p0 + w_outer;
		const auto opening_p1_inner = opening_p1 + w_inner;
		const auto opening_p1_outer = opening_p1 + w_outer;

		const float z_bottom = opening.z_min;
		const float z_top = opening.z_max;
		const float opening_height = z_top - z_bottom;

		// 1. LEFT REVEAL (vertical surface at left edge of opening)
		addQuad(
			{opening_p0_outer.x, opening_p0_outer.y, z_bottom},
			{opening_p0_inner.x, opening_p0_inner.y, z_bottom},
			{opening_p0_inner.x, opening_p0_inner.y, z_top},
			{opening_p0_outer.x, opening_p0_outer.y, z_top}, 0.0f, 0.0f, thickness_ / textureSizeX_,
			opening_height / textureSizeY_);

		// 2. RIGHT REVEAL (vertical surface at right edge of opening)
		addQuad(
			{opening_p1_inner.x, opening_p1_inner.y, z_bottom},
			{opening_p1_outer.x, opening_p1_outer.y, z_bottom},
			{opening_p1_outer.x, opening_p1_outer.y, z_top},
			{opening_p1_inner.x, opening_p1_inner.y, z_top}, 0.0f, 0.0f, thickness_ / textureSizeX_,
			opening_height / textureSizeY_);

		// 3. TOP REVEAL (horizontal surface at top of opening)
		addQuad(
			{opening_p0_inner.x, opening_p0_inner.y, z_top},
			{opening_p1_inner.x, opening_p1_inner.y, z_top},
			{opening_p1_outer.x, opening_p1_outer.y, z_top},
			{opening_p0_outer.x, opening_p0_outer.y, z_top}, 0.0f, 0.0f,
			opening.width / textureSizeX_, thickness_ / textureSizeY_);

		// 4. BOTTOM REVEAL (horizontal surface at bottom of opening)
		addQuad(
			{opening_p0_outer.x, opening_p0_outer.y, z_bottom},
			{opening_p1_outer.x, opening_p1_outer.y, z_bottom},
			{opening_p1_inner.x, opening_p1_inner.y, z_bottom},
			{opening_p0_inner.x, opening_p0_inner.y, z_bottom}, 0.0f, 0.0f,
			opening.width / textureSizeX_, thickness_ / textureSizeY_);
	}
}

void VerticalPlane::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
}

void VerticalPlane::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
}