// Fernando Cañadas Aránega, 5 Nov 2025

#pragma once

#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/TParameterDefinitions.h>

#include <string>
#include <vector>

namespace mvsim
{
/** Fricción adaptativa: cambia mu y C_damping según la zona del mapa.
 * \note Remove once the world implements ground properties by zone
 * \ingroup friction_module
 */
class AdaptativeFriction : public FrictionBase
{
	DECLARES_REGISTER_FRICTION(AdaptativeFriction)

   public:
	AdaptativeFriction(VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	virtual mrpt::math::TVector2D evaluate_friction(
		const FrictionBase::TFrictionInput& input) const override;

	double getMu() const { return mu_; }
	double getCdamping() const { return C_damping_; }
	double getCrr() const { return Crr_; }
	double getLongitudinalForce() const { return F_friction_lon_act; }

	struct ZoneParams
	{
		std::string name;
		double mu;
		double C_damping;
		double Crr;
		double x_min, x_max, y_min, y_max;
	};
	const std::vector<ZoneParams>& getZones() const { return zones_; }

   private:
	double mu_;	 //!< Coeficiente de fricción
	double C_damping_;	//!< Amortiguamiento interno de la rueda
	double Crr_;
	std::vector<ZoneParams> zones_;
};
}  // namespace mvsim
