/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mvsim/WorldElements/WorldElementBase.h>

namespace mvsim
{
/** A 2D grid without any physical implications, just visualization purposes
 *  \ingroup world_elements_module
 */
class GroundGrid : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(GroundGrid)
   public:
	GroundGrid(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~GroundGrid();

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::viz::Scene>& viz,
		const mrpt::optional_ref<mrpt::viz::Scene>& physical, bool childrenOnly) override;

	bool is_floating_;
	std::string float_center_at_vehicle_name_;
	double x_min_, x_max_, y_min_, y_max_, interval_;
	mrpt::img::TColor color_;
	double line_width_;

	mrpt::viz::CGridPlaneXY::Ptr gl_groundgrid_;
};
}  // namespace mvsim
