/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/poses/CPose3D.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mvsim
{
/** Linear suspension of a vehicle chassis (the sprung mass) on the body that
 *  carries its wheels.
 *
 * The chassis has three degrees of freedom relative to the wheels' body:
 * heave (z, up), roll (left side up) and pitch (nose down), measured from the
 * static equilibrium. It rotates about a pivot point on the ground, under its
 * center of mass, and rests on a spring and a damper at each corner. The
 * equations of motion are linear, so they are integrated exactly for any time
 * step (see the physics documentation).
 *
 * Degrees of freedom without stiffness (e.g. the pitch of a vehicle whose
 * wheels all lie on one axle) are locked: the chassis follows the wheels.
 *
 * \ingroup mvsim_simulator_module
 */
class Suspension
{
   public:
	/// (heave, roll, pitch) values
	using Vector3 = mrpt::math::CVectorFixedDouble<3>;

	/// Indices of the degrees of freedom in all 3-vectors of this class:
	enum Dof : int
	{
		HEAVE = 0,
		ROLL = 1,
		PITCH = 2
	};

	/// User parameters, from the vehicle definition <suspension> block
	struct Config
	{
		bool enabled = false;
		double frequency_hz = 1.5;	//!< Heave natural frequency, used for corners without stiffness
		double damping_ratio = 0.3;	 //!< Used for corners without damping
		std::map<size_t, double> stiffness;	 //!< Per-corner (wheel index) stiffness [N/m]
		std::map<size_t, double> damping;  //!< Per-corner (wheel index) damping [N.s/m]
		std::optional<double> I_xx, I_yy;  //!< Chassis inertia about its COM [kg.m^2]
		std::optional<double> cog_height;  //!< Chassis COM height over the ground [m]
	};

	struct Corner
	{
		mrpt::math::TPoint2D pos;  //!< In the vehicle frame
		double stiffness = 0;  //!< [N/m]
		double damping = 0;	 //!< [N.s/m]
	};

	struct Params
	{
		double sprungMass = 0;	//!< [kg]
		mrpt::math::TPoint2D com;  //!< Sprung mass center, in the vehicle frame
		double comHeight = 0;  //!< Over the ground (the pivot) [m]
		double I_xx = 0;  //!< About the COM [kg.m^2]
		double I_yy = 0;  //!< About the COM [kg.m^2]
		double gravity = 9.81;
		std::vector<Corner> corners;
	};

	/** Builds the model.
	 * \exception std::exception If the chassis would be unstable (springs too
	 * soft for the height of its center of mass) or there are no corners. */
	void init(const Params& p);

	bool initialized() const { return initialized_; }
	const Params& params() const { return p_; }

	/** Advances the state by dt, with constant inputs during the step:
	 * \param baseRates Velocity of the wheels' body: along its normal [m/s],
	 * and roll and pitch rates [rad/s].
	 * \param accel Acceleration of the vehicle in its own frame [m/s^2].
	 */
	void step(double dt, const Vector3& baseRates, const mrpt::math::TVector2D& accel);

	/// Chassis displacement (heave, roll, pitch) relative to the wheels' body
	const Vector3& displacement() const { return delta_; }

	/// Time derivative of displacement()
	Vector3 relativeVelocity() const;

	/// Chassis absolute (heave, roll, pitch) velocity: relativeVelocity() plus base rates
	const Vector3& absoluteVelocity() const { return w_; }

	/// Average absolute acceleration of the chassis in the last step
	const Vector3& absoluteAcceleration() const { return acc_; }

	/** Pose of the chassis relative to the vehicle frame (that of the wheels'
	 *  body), from displacement() */
	mrpt::poses::CPose3D relativePose() const;

	/** For each corner, the change of the vertical force of its spring and
	 *  damper with respect to the static equilibrium [N] (positive: more
	 *  load on its wheel). */
	std::vector<double> cornerLoadChanges() const;

	struct Mode
	{
		double frequency_hz = 0;  //!< Undamped natural frequency
		double damping_ratio = 0;
		std::string shape;	//!< Dominant degree of freedom: heave, roll or pitch
	};

	/// Vibration modes of the chassis, from the eigenvalues of its equations
	const std::vector<Mode>& modes() const { return modes_; }

	/// e.g. "heave 1.50 Hz (zeta=0.30), pitch 1.71 Hz (zeta=0.34)"
	std::string modesAsString() const;

	/// Sets the state (e.g. for an initial perturbation)
	void setState(const Vector3& displacement, const Vector3& absoluteVelocity);

   private:
	bool initialized_ = false;
	Params p_;
	std::vector<int> active_;  //!< Unlocked degrees of freedom
	mrpt::math::CMatrixDouble J_;  //!< Corner deflections from displacement: (#corners x 3)
	//! Reduced to the active degrees of freedom:
	mrpt::math::CMatrixDouble M_;
	mrpt::math::CMatrixDouble K_;
	mrpt::math::CMatrixDouble C_;

	// Exact discretization for the cached time step:
	double cachedDt_ = 0;
	mrpt::math::CMatrixDouble Phi_;
	mrpt::math::CMatrixDouble Gamma_;

	Vector3 delta_ = Vector3::Zero();
	Vector3 w_ = Vector3::Zero();
	Vector3 acc_ = Vector3::Zero();
	Vector3 baseRates_ = Vector3::Zero();  //!< Of the last step

	std::vector<Mode> modes_;

	void updateDiscretization(double dt);
	void computeModes();
};

}  // namespace mvsim
