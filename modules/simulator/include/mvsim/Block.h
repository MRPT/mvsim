/*+-------------------------------------------------------------------------+
  |                       Multiblock simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_fixture.h>
#include <box2d/b2_friction_joint.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_world.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/Sensors/SensorBase.h>
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
	static void register_block_class(const World& parent, const rapidxml::xml_node<char>* xml_node);

	// ------- Interface with "World" ------
	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;
	virtual void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint = mrpt::math::TPoint2D(0, 0)) override;

	/** Create bodies, fixtures, etc. for the dynamical simulation. May be
	 * overrided by derived classes */
	virtual void create_multibody_system(b2World& world);

	/** Get (an approximation of) the max radius of the block, from its point of
	 * reference (in meters) */
	virtual float getMaxBlockRadius() const { return maxRadius_; }
	/** Get the block mass */
	virtual double getMass() const { return mass_; }
	b2Body* getBox2DBlockBody() { return b2dBody_; }
	mrpt::math::TPoint2D getBlockCenterOfMass() const
	{
		return block_com_;
	}  //!< In local coordinates

	/** Get the 2D shape of the block, as set from the config file (only used
	 * for collision detection) */
	const mrpt::math::TPolygon2D& blockShape() const { return block_poly_; }

	void blockShape(const mrpt::math::TPolygon2D& p)
	{
		block_poly_ = p;
		updateMaxRadiusFromPoly();
		gl_block_.reset();	// regenerate 3D view
	}

	/** Set the block index in the World */
	void setBlockIndex(size_t idx) { blockIndex_ = idx; }
	/** Get the block index in the World */
	size_t getBlockIndex() const { return blockIndex_; }

	Block(World* parent);

	double ground_friction() const { return groundFriction_; }
	void ground_friction(double newValue) { groundFriction_ = newValue; }

	double mass() const { return mass_; }
	void mass(double newValue) { mass_ = newValue; }

	bool isStatic() const;
	void setIsStatic(bool b);

	const mrpt::img::TColor block_color() const { return block_color_; }
	void block_color(const mrpt::img::TColor& c)
	{
		block_color_ = c;
		gl_block_.reset();	// regenerate 3D view
	}

	double block_z_min() const { return block_z_min_; }
	double block_z_max() const { return block_z_max_; }
	void block_z_min(double v)
	{
		block_z_min_ = v;
		gl_block_.reset();	// regenerate 3D view
	}
	void block_z_max(double v)
	{
		block_z_max_ = v;
		gl_block_.reset();	// regenerate 3D view
	}

	/// returns true if none of the min/max block z limits has been set
	/// explicitly yet. Used while parsing the shape_from_visual tag.
	bool default_block_z_min_max() const;

	VisualObject* meAsVisualObject() override { return this; }

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	/** user-supplied index number: must be set/get'ed with setblockIndex()
	 * getblockIndex() (default=0) */
	size_t blockIndex_ = 0;

	std::vector<b2FrictionJoint*> friction_joints_;

	// Block info:
	double mass_ = 30.0;
	bool isStatic_ = false;
	mrpt::math::TPolygon2D block_poly_;

	/// Automatically computed from block_poly_ upon each change via
	/// updateMaxRadiusFromPoly()
	double maxRadius_;

	double block_z_min_ = std::numeric_limits<double>::quiet_NaN(),
		   block_z_max_ = std::numeric_limits<double>::quiet_NaN();

	mrpt::img::TColor block_color_{0x00, 0x00, 0xff};
	mrpt::math::TPoint2D block_com_{.0, .0};  //!< In local coordinates

	double lateral_friction_ = 0.5;	 //!< Default: 0.5
	double groundFriction_ = 0.5;  //!< Default: 0.5
	double restitution_ = 0.01;	 //!< Default: 0.01

	/** If intangible, a block will be rendered visually but will be neither
	 * detected by sensors, nor collide  */
	bool intangible_ = false;

	const TParameterDefinitions params_ = {
		{"mass", {"%lf", &mass_}},
		{"zmin", {"%lf", &block_z_min_}},
		{"zmax", {"%lf", &block_z_max_}},
		{"ground_friction", {"%lf", &groundFriction_}},
		{"lateral_friction", {"%lf", &lateral_friction_}},
		{"restitution", {"%lf", &restitution_}},
		{"color", {"%color", &block_color_}},
		{"intangible", {"%bool", &intangible_}},
		{"static", {"%bool", &isStatic_}}
		//
	};

	void updateMaxRadiusFromPoly();

	// Box2D elements:
	b2Fixture* fixture_block_;

   private:
	void internal_internalGuiUpdate_forces(mrpt::opengl::COpenGLScene& scene);

	void internal_parseGeometry(const rapidxml::xml_node<char>& xml_geom_node);

	mrpt::opengl::CSetOfObjects::Ptr gl_block_;
	mrpt::opengl::CSetOfLines::Ptr gl_forces_;
	std::mutex force_segments_for_rendering_cs_;
	std::vector<mrpt::math::TSegment3D> force_segments_for_rendering_;

};	// end Block

/** An invisible block which can be used as an auxiliary anchor object. */
class DummyInvisibleBlock : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<DummyInvisibleBlock>;

	DummyInvisibleBlock(World* parent);

	/** Class factory: Creates a block from XML description of type
	 * "<block>...</block>".  */
	static Ptr factory(World* parent, const rapidxml::xml_node<char>* xml_node);
	/// \overload
	static Ptr factory(World* parent, const std::string& xml_text);

	// ------- Interface with "World" ------
	virtual void simul_pre_timestep(const TSimulContext& context) override
	{
		Simulable::simul_pre_timestep(context);
		for (auto& s : sensors_) s->simul_pre_timestep(context);
	}
	virtual void simul_post_timestep(const TSimulContext& context) override
	{
		Simulable::simul_post_timestep(context);
		for (auto& s : sensors_) s->simul_post_timestep(context);
	}

	virtual void apply_force(
		[[maybe_unused]] const mrpt::math::TVector2D& force,
		[[maybe_unused]] const mrpt::math::TPoint2D& applyPoint) override
	{
	}

	virtual void create_multibody_system(b2World&) {}

	virtual float getMaxBlockRadius() const { return 0; }

	/** Get the block mass */
	virtual double getMass() const { return 0; }

	void add_sensor(const SensorBase::Ptr& sensor) { sensors_.push_back(sensor); }

   protected:
	void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		[[maybe_unused]] bool childrenOnly) override;

	void registerOnServer(mvsim::Client& c) override
	{
		// register myself, and my children objects:
		Simulable::registerOnServer(c);
		for (auto& sensor : sensors_) sensor->registerOnServer(c);
	}

   private:
	TListSensors sensors_;	//!< Sensors aboard

};	// end Block

}  // namespace mvsim
