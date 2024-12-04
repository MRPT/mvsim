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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VisualObject.h>
#include <mvsim/basic_types.h>

namespace mvsim
{
class DefaultFriction;
class VehicleBase;
class DynamicsDifferential;
class World;

/** Common info for 2D wheels, for usage in derived classes.
 * Wheels are modeled as a mass with a cylindrical shape.
 *
 *  \ingroup mvsim_simulator_module
 */
class Wheel : public VisualObject
{
   public:
	Wheel(World* world);

	/** Location of the wheel wrt the chassis ref point [m,rad] (in local
	 * coords) */
	double x = 0, y = -0.5, yaw = 0;

	/** Pose of the wheel wrt the chassis ref point (in local coords) */
	mrpt::math::TPose3D pose() const { return {x, y, 0, yaw, 0, 0}; }

	/** Length(diameter) and width of the wheel rectangle [m] */
	double diameter = .4, width = .2;
	double mass = 2.0;	//!< [kg]

	/** Inertia: computed automatically from geometry at constructor and at \a
	 * loadFromXML(), but can be overrided. */
	double Iyy = 1.0;

	/** Color for OpenGL rendering */
	mrpt::img::TColor color{0xff323232};

	/** Optional: name of a named custom visualization object in my parent
	 * vehicle, whose angle (yaw) is to be set whenever this wheel angle is
	 * updated.
	 */
	std::string linked_yaw_object_name;
	double linked_yaw_offset = .0;

	const TParameterDefinitions params_ = {
		{"mass", {"%lf", &mass}},
		{"width", {"%lf", &width}},
		{"diameter", {"%lf", &diameter}},
		{"color", {"%color", &color}},
		{"inertia", {"%lf", &Iyy}},
		{"linked_yaw", {"%s", &linked_yaw_object_name}},
		{"linked_yaw_offset_deg", {"%lf_deg", &linked_yaw_offset}}};

	/** Generates a human-readable description of the wheel parameters and
	 * kinematic status */
	std::string asString() const;

	// methods ----

	void getAs3DObject(mrpt::opengl::CSetOfObjects& obj, bool isPhysicalScene);
	void loadFromXML(const rapidxml::xml_node<char>* xml_node);

	void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	double getPhi() const { return phi; }  //!< Orientation (rad) wrt vehicle local frame
	void setPhi(double val) { phi = val; }	//!< Orientation (rad) wrt vehicle local frame
	double getW() const { return w; }  //!< Spinning velocity (rad/s) wrt shaft
	void setW(double val) { w = val; }	//!< Spinning velocity (rad/s) wrt shaft
	void recalcInertia();  //!< Recompute Iyy from mass, diameter and height.
   protected:
	/** Angular position and velocity of the wheel as it spins over its shaft
	 * (rad, rad/s) */
	double phi = 0, w = 0;
};
}  // namespace mvsim
