/*+-------------------------------------------------------------------------+
  |                       Multiblock simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/Simulable.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VisualObject.h>
#include <mvsim/basic_types.h>

#include <mutex>

namespace mvsim
{
/** A non-vehicle "actor" for the simulation, typically obstacle blocks.
 */
class Block : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<Block>;

	/** Class factory: Creates a block from XML description of type
	 * "<block>...</block>".  */
	static Ptr factory(World* parent, const rapidxml::xml_node<char>* xml_node);
	/// \overload
	static Ptr factory(World* parent, const std::string& xml_text);

	/** Register a new class of blocks from XML description of type
	 * "<block:class name='name'>...</block:class>".  */
	static void register_block_class(const rapidxml::xml_node<char>* xml_node);

	// ------- Interface with "World" ------
	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;
	virtual void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint =
			mrpt::math::TPoint2D(0, 0)) override;

	/** Create bodies, fixtures, etc. for the dynamical simulation. May be
	 * overrided by derived classes */
	virtual void create_multibody_system(b2World& world);

	/** Get (an approximation of) the max radius of the block, from its point of
	 * reference (in meters) */
	virtual float getMaxBlockRadius() const { return m_max_radius; }
	/** Get the block mass */
	virtual double getMass() const { return m_mass; }
	b2Body* getBox2DBlockBody() { return m_b2d_body; }
	mrpt::math::TPoint2D getBlockCenterOfMass() const
	{
		return m_block_com;
	}  //!< In local coordinates

	/** Get the 2D shape of the block, as set from the config file (only used
	 * for collision detection) */
	const mrpt::math::TPolygon2D& blockShape() const { return m_block_poly; }

	void blockShape(const mrpt::math::TPolygon2D& p)
	{
		m_block_poly = p;
		updateMaxRadiusFromPoly();
		m_gl_block.reset();  // regenerate 3D view
	}

	/** Set the block index in the World */
	void setBlockIndex(size_t idx) { m_block_index = idx; }
	/** Get the block index in the World */
	size_t getBlockIndex() const { return m_block_index; }

	Block(World* parent);

	double ground_friction() const { return m_ground_friction; }
	void ground_friction(double newValue) { m_ground_friction = newValue; }

	double mass() const { return m_mass; }
	void mass(double newValue) { m_mass = newValue; }

	bool isStatic() const;
	void setIsStatic(bool b);

	void poses_mutex_lock() override { m_gui_mtx.lock(); }
	void poses_mutex_unlock() override { m_gui_mtx.unlock(); }

	const mrpt::img::TColor block_color() const { return m_block_color; }
	void block_color(const mrpt::img::TColor& c)
	{
		m_block_color = c;
		m_gl_block.reset();  // regenerate 3D view
	}

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& scene, bool childrenOnly) override;
	virtual mrpt::poses::CPose3D internalGuiGetVisualPose() override;

	/** user-supplied index number: must be set/get'ed with setblockIndex()
	 * getblockIndex() (default=0) */
	size_t m_block_index = 0;

	std::vector<b2FrictionJoint*> m_friction_joints;

	// Block info:
	double m_mass = 30.0;
	bool m_isStatic = false;
	mrpt::math::TPolygon2D m_block_poly;
	double m_max_radius;  //!< Automatically computed from m_block_poly upon
						  //! each change via updateMaxRadiusFromPoly()
	double m_block_z_min = 0.0, m_block_z_max = 1.0;
	mrpt::img::TColor m_block_color{0x00, 0x00, 0xff};
	mrpt::math::TPoint2D m_block_com{.0, .0};  //!< In local coordinates

	double m_lateral_friction = 0.5;  //!< Default: 0.5
	double m_ground_friction = 0.5;  //!< Default: 0.5
	double m_restitution = 0.01;  //!< Deault: 0.01

	const TParameterDefinitions m_params = {
		{"mass", {"%lf", &m_mass}},
		{"zmin", {"%lf", &m_block_z_min}},
		{"zmax", {"%lf", &m_block_z_max}},
		{"ground_friction", {"%lf", &m_ground_friction}},
		{"lateral_friction", {"%lf", &m_lateral_friction}},
		{"restitution", {"%lf", &m_restitution}},
		{"color", {"%color", &m_block_color}}};

	void updateMaxRadiusFromPoly();

	// Box2D elements:
	b2Fixture* m_fixture_block;

   private:
	void internal_internalGuiUpdate_forces(mrpt::opengl::COpenGLScene& scene);

	mrpt::opengl::CSetOfObjects::Ptr m_gl_block;
	mrpt::opengl::CSetOfLines::Ptr m_gl_forces;
	std::mutex m_force_segments_for_rendering_cs;
	std::vector<mrpt::math::TSegment3D> m_force_segments_for_rendering;

	std::mutex m_gui_mtx;

};  // end Block
}  // namespace mvsim
