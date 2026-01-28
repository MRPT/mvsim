/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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
/** A vertical plane (or wall with thickness) visible from one or both sides (see "cull_faces"),
 *  conveniently defined by starting and end points (x0,y0)-(x1,y1).
 *  Supports doors and windows with proper collision handling and rendering.
 * \ingroup world_elements_module
 */
class VerticalPlane : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(VerticalPlane)
   public:
	VerticalPlane(World* parent, const rapidxml::xml_node<char>* root);

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;
	// ------- Interface with "World" ------
	void simul_pre_timestep(const TSimulContext& context) override;
	void simul_post_timestep(const TSimulContext& context) override;

	/** Structure to represent an opening (door or window) in the wall */
	struct Opening
	{
		enum class Type
		{
			DOOR,  // No collision, allows passage
			WINDOW	// Has collision, but visible opening
		};

		Type type = Type::DOOR;
		float position = 0.0f;	//!< Position along wall length [0,1] (0=start, 1=end)
		float width = 1.0f;	 //!< Width of opening in meters
		float z_min = 0.0f;	 //!< Bottom height of opening (meters)
		float z_max = 2.1f;	 //!< Top height of opening (meters)
		std::string name;  //!< Optional name for the opening

		/** Check if this opening overlaps with a given position range */
		bool overlaps(float pos_start, float pos_end) const;
	};

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	/** Create Box2D physics bodies for the wall segments */
	void createPhysicsBodies();

	/** Create visual representation with proper segmentation for openings */
	void createVisualRepresentation(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical);

	/** Render a front/back face of the wall */
	void renderWallFace(
		const mrpt::math::TPoint2Df& p0, const mrpt::math::TPoint2Df& p1,
		const mrpt::math::TPoint2Df& normal_offset, bool is_front_face,
		const mrpt::img::CImage* texture = nullptr);

	/** Render top/bottom/side edges of thick wall */
	void renderWallEdges(
		const mrpt::math::TPoint2Df& p0, const mrpt::math::TPoint2Df& p1,
		const mrpt::math::TPoint2Df& w_inner, const mrpt::math::TPoint2Df& w_outer,
		const mrpt::img::CImage* texture = nullptr);

	/** Render the reveals (inner edges) around door and window openings */
	void renderOpeningReveals(
		const mrpt::math::TPoint2Df& p0, const mrpt::math::TPoint2Df& p1,
		const mrpt::math::TPoint2Df& dir, const mrpt::math::TPoint2Df& w_inner,
		const mrpt::math::TPoint2Df& w_outer, float wall_length,
		const mrpt::img::CImage* texture = nullptr);

	float x0_ = -10, x1_ = -10, y0_ = -10, y1_ = 10;
	mrpt::img::TColor color_ = {0xa0, 0xa0, 0xa0, 0xff};
	bool enableShadows_ = true;

	/** Wall thickness (default: 0.02m - thin wall, can be increased for thick walls) */
	float thickness_ = 0.02f;

	/** If defined, it overrides the plain color in color_ */
	std::string textureFileName_;
	double textureSizeX_ = 1.0;
	double textureSizeY_ = 1.0;

	float z_ = .0f, height_ = 3.0f;
	std::string cull_faces_ = "NONE";

	mrpt::opengl::CTexturedPlane::Ptr gl_plane_;
	mrpt::opengl::CSetOfTexturedTriangles::Ptr gl_plane_text_;
	mrpt::opengl::CSetOfObjects::Ptr glGroup_;

	/** Box2D body for the wall (may be split into multiple fixtures) */
	b2Body* b2dBody_ = nullptr;
	double restitution_ = 0.01;	 //!< Default: 0.01

	/** Multiple fixtures for wall segments (to exclude door openings) */
	std::vector<b2Fixture*> fixtures_;

	/** List of openings (doors and windows) in this wall */
	std::vector<Opening> openings_;
};
}  // namespace mvsim