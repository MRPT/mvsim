/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/Sensors/ImuNoiseModel.h>

#include <cmath>

using namespace mvsim;

void ImuNoiseModel::reset()
{
	gyroBias_ = {0.0, 0.0, 0.0};
	accelBias_ = {0.0, 0.0, 0.0};
}

mrpt::math::TVector3D ImuNoiseModel::applyGyroscope(const mrpt::math::TVector3D& trueVal, double dt)
{
	return applyChannel(trueVal, dt, gyroscope, gyroBias_);
}

mrpt::math::TVector3D ImuNoiseModel::applyAccelerometer(
	const mrpt::math::TVector3D& trueVal, double dt)
{
	return applyChannel(trueVal, dt, accelerometer, accelBias_);
}

mrpt::math::TVector3D ImuNoiseModel::applyChannel(
	const mrpt::math::TVector3D& trueVal, double dt, const ChannelParams& params,
	mrpt::math::TVector3D& bias)
{
	// 1) Integrate bias random walk:
	//    b_k = b_{k-1} + N(0, sigma_rw^2 * dt) per axis
	if (params.random_walk_std > 0.0)
	{
		const double rwSigma = params.random_walk_std * std::sqrt(dt);
		bias.x += rng_.drawGaussian1D(0.0, rwSigma);
		bias.y += rng_.drawGaussian1D(0.0, rwSigma);
		bias.z += rng_.drawGaussian1D(0.0, rwSigma);
	}

	// 2) Add white noise + bias to the true value:
	//    m_k = true_k + b_k + N(0, sigma_w^2)
	mrpt::math::TVector3D noisy = trueVal + bias;

	if (params.white_noise_std > 0.0)
	{
		noisy.x += rng_.drawGaussian1D(0.0, params.white_noise_std);
		noisy.y += rng_.drawGaussian1D(0.0, params.white_noise_std);
		noisy.z += rng_.drawGaussian1D(0.0, params.white_noise_std);
	}

	return noisy;
}
