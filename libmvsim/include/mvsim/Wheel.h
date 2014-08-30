/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mvsim/basic_types.h>

#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/TColor.h>

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
		double x,y,yaw; //!< Location of the wheel wrt the chassis ref point [m,rad] (in local coords)
		double diameter,width; //!< Length(diameter) and width of the wheel rectangle [m]
		double mass; //!< [kg]
		double Iyy;  //!< Inertia: computed automatically from geometry at constructor and at \a loadFromXML(), but can be overrided.
		mrpt::utils::TColor color; //!< Color for OpenGL rendering

		Wheel();
		void getAs3DObject(mrpt::opengl::CSetOfObjects &obj);
		void loadFromXML(const rapidxml::xml_node<char> *xml_node);

		double getPhi() const {return phi;} //!< Orientation (rad) wrt vehicle local frame
		void setPhi(double val) {phi=val;}  //!< Orientation (rad) wrt vehicle local frame
		double getW() const {return w;}  //!< Spinning velocity (rad/s) wrt shaft
		void setW(double val) {w=val;} //!< Spinning velocity (rad/s) wrt shaft

		void recalcInertia(); //!< Recompute Iyy from mass, diameter and height. 
	protected:
		double phi, w; //!< Angular position and velocity of the wheel as it spins over its shaft (rad, rad/s)

	};

}
