/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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
#include <mvsim/basic_types.h>

namespace mvsim
{
class DefaultFriction;
class VehicleBase;
class DynamicsDifferential;

/** Common info for 2D wheels, for usage in derived classes.
 * Wheels are modeled as a mass with a rectangular shape.
 */
class Wheel
{
   public:
	Wheel();

	/** Location of the wheel wrt the chassis ref point [m,rad] (in local
	 * coords) */
	double x = 0, y = -0.5, yaw = 0;

	/** Length(diameter) and width of the wheel rectangle [m] */
	double diameter = .4, width = .2;
	double mass = 2.0;	//!< [kg]

	/** Inertia: computed automatically from geometry at constructor and at \a
	 * loadFromXML(), but can be overrided. */
	double Iyy = 1.0;

	/** Color for OpenGL rendering */
	mrpt::img::TColor color{0xff323232};

	const TParameterDefinitions m_params = {
		{"mass", {"%lf", &mass}},
		{"width", {"%lf", &width}},
		{"diameter", {"%lf", &diameter}},
		{"color", {"%color", &color}},
		{"inertia", {"%lf", &Iyy}}};

	// methods ----

	void getAs3DObject(mrpt::opengl::CSetOfObjects& obj);
	void loadFromXML(const rapidxml::xml_node<char>* xml_node);

	double getPhi() const
	{
		return phi;
	}  //!< Orientation (rad) wrt vehicle local frame
	void setPhi(double val)
	{
		phi = val;
	}  //!< Orientation (rad) wrt vehicle local frame
	double getW() const { return w; }  //!< Spinning velocity (rad/s) wrt shaft
	void setW(double val) { w = val; }	//!< Spinning velocity (rad/s) wrt shaft
	void recalcInertia();  //!< Recompute Iyy from mass, diameter and height.
   protected:
	/** Angular position and velocity of the wheel as it spins over its shaft
	 * (rad, rad/s) */
	double phi = 0, w = 0;
};
}  // namespace mvsim
