/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/random.h>

namespace mvsim
{
/**
 * \brief Stochastic IMU error model based on Forster et al. (2016).
 *
 * Implements the standard continuous-time IMU noise model with two
 * components per axis for both gyroscope and accelerometer channels:
 *
 *  - **White noise** (measurement noise / angle/velocity random walk):
 *    Additive zero-mean Gaussian noise applied to each sample.
 *
 *  - **Bias random walk** (in-run stability / drift):
 *    A slowly-varying bias modelled as the integral of white noise,
 *    i.e. a Wiener process.
 *
 * Discrete-time update equations (timestep \f$\Delta t\f$):
 * \f[
 *   \mathbf{b}_{k} = \mathbf{b}_{k-1}
 *       + \mathcal{N}\!\bigl(\mathbf{0},\,\sigma_{rw}^{2}\,\Delta t\,
 *         \mathbf{I}\bigr)
 * \f]
 * \f[
 *   \tilde{\mathbf{m}}_{k} = \mathbf{m}_{k}
 *       + \mathbf{b}_{k}
 *       + \mathcal{N}\!\bigl(\mathbf{0},\,\sigma_{w}^{2}\,\mathbf{I}\bigr)
 * \f]
 *
 * where \f$\sigma_{w}\f$ is the white-noise standard deviation and
 * \f$\sigma_{rw}\f$ is the random-walk standard deviation.
 *
 * \par Reference
 * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza,
 * "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry",
 * IEEE Transactions on Robotics, vol. 33, no. 1, pp. 1-21, 2016.
 *
 * \ingroup sensors_module
 */
class ImuNoiseModel
{
   public:
	/** \brief Parameters for a single 3-axis IMU channel. */
	struct ChannelParams
	{
		/** White-noise standard deviation (same units as the channel). */
		double white_noise_std = 0.0;

		/** Bias random-walk standard deviation
		 *  (units / sqrt(s) in continuous time). */
		double random_walk_std = 0.0;
	};

	/** Gyroscope noise parameters [rad/s]. */
	ChannelParams gyroscope;

	/** Accelerometer noise parameters [m/s²]. */
	ChannelParams accelerometer;

	/** \brief Reset the internal bias states to zero. */
	void reset();

	/**
	 * \brief Corrupt a true gyroscope reading with noise.
	 *
	 * \param[in] trueVal True angular velocity (rad/s).
	 * \param[in] dt      Simulation timestep (s).
	 * \return Noisy angular velocity (rad/s).
	 */
	mrpt::math::TVector3D applyGyroscope(const mrpt::math::TVector3D& trueVal, double dt);

	/**
	 * \brief Corrupt a true accelerometer reading with noise.
	 *
	 * \param[in] trueVal True linear acceleration (m/s²).
	 * \param[in] dt      Simulation timestep (s).
	 * \return Noisy linear acceleration (m/s²).
	 */
	mrpt::math::TVector3D applyAccelerometer(const mrpt::math::TVector3D& trueVal, double dt);

	/** Read-only access to current gyroscope bias (for diagnostics). */
	const mrpt::math::TVector3D& gyroBias() const { return gyroBias_; }

	/** Read-only access to current accelerometer bias (for diagnostics). */
	const mrpt::math::TVector3D& accelBias() const { return accelBias_; }

   private:
	/** Apply the two-component noise model to a 3-axis signal.
	 *
	 * \param[in] trueVal   Ground-truth value.
	 * \param[in] dt        Timestep (s).
	 * \param[in] params    Channel parameters.
	 * \param[in,out] bias  Current bias state (updated in place).
	 * \return Noisy measurement.
	 */
	mrpt::math::TVector3D applyChannel(
		const mrpt::math::TVector3D& trueVal, double dt, const ChannelParams& params,
		mrpt::math::TVector3D& bias);

	mrpt::math::TVector3D gyroBias_{0.0, 0.0, 0.0};
	mrpt::math::TVector3D accelBias_{0.0, 0.0, 0.0};

	mrpt::random::CRandomGenerator rng_;
};

}  // namespace mvsim
