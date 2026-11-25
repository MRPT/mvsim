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

mrpt::math::CMatrixDouble33 ImuNoiseModel::ChannelParams::sensitivityMatrix() const
{
	const auto& m = misalignment;
	const mrpt::math::TVector3D& s = scale;
	mrpt::math::CMatrixDouble33 T;
	T(0, 0) = s.x;
	T(0, 1) = s.x * m[0];
	T(0, 2) = s.x * m[1];
	T(1, 0) = s.y * m[2];
	T(1, 1) = s.y;
	T(1, 2) = s.y * m[3];
	T(2, 0) = s.z * m[4];
	T(2, 1) = s.z * m[5];
	T(2, 2) = s.z;
	return T;
}

bool ImuNoiseModel::ChannelParams::hasIdealSensitivity() const
{
	if (scale.x != 1.0 || scale.y != 1.0 || scale.z != 1.0)
	{
		return false;
	}
	for (const double m : misalignment)
	{
		if (m != 0.0)
		{
			return false;
		}
	}
	return true;
}

void ImuNoiseModel::reset()
{
	gyroBias_ = gyroscope.initial_bias;
	accelBias_ = accelerometer.initial_bias;
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

	// 2) Add white noise + bias to the true value, scaled and misaligned:
	//    m_k = T * true_k + b_k + N(0, sigma_w^2)
	mrpt::math::TVector3D noisy = trueVal;
	if (!params.hasIdealSensitivity())
	{
		const auto T = params.sensitivityMatrix();
		noisy.x = T(0, 0) * trueVal.x + T(0, 1) * trueVal.y + T(0, 2) * trueVal.z;
		noisy.y = T(1, 0) * trueVal.x + T(1, 1) * trueVal.y + T(1, 2) * trueVal.z;
		noisy.z = T(2, 0) * trueVal.x + T(2, 1) * trueVal.y + T(2, 2) * trueVal.z;
	}
	noisy = noisy + bias;

	if (params.white_noise_std > 0.0)
	{
		noisy.x += rng_.drawGaussian1D(0.0, params.white_noise_std);
		noisy.y += rng_.drawGaussian1D(0.0, params.white_noise_std);
		noisy.z += rng_.drawGaussian1D(0.0, params.white_noise_std);
	}

	return noisy;
}
