/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "ImuSimulator.h"

#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/Lie/SO.h>
#include <mvsim/World.h>

#include <algorithm>

#include "TimeUtils.h"

using namespace mvsim_dataset_gen;
using mrpt::math::TPoint3D;
using mrpt::math::TVector3D;
using mrpt::poses::CPose3D;

mrpt::obs::CObservationIMU::Ptr mvsim_dataset_gen::simulateImuSample(
	const mvsim::World& world, const TrajectorySource& traj, const CPose3D& sensorPoseOnVehicle,
	double tEpochSeconds, double diffStep, double sampleDt, const std::string& sensorLabel,
	bool measureOrientation, double orientationStdNoise, mvsim::ImuNoiseModel& noiseModel,
	std::mt19937& rng)
{
	// The stencil is evaluated at exact clock ticks, so that its three
	// points are exactly equally spaced even with epoch timestamps.
	const auto hTicks = mrpt::Clock::duration(std::max<int64_t>(1, std::llround(diffStep * 1e7)));
	const double h = 1e-7 * static_cast<double>(hTicks.count());

	// The stencil below needs [tc-h, tc+h] inside the trajectory's own time
	// span: TrajectorySource::poseAt() clamps queries outside it, and
	// clamping just one side of a *central* difference (rather than
	// rejecting the query) turns it into an invalid, wildly wrong estimate.
	// Evaluate the stencil at a `tc` nudged in by at most `h` from either
	// edge instead; the sample's own reported timestamp is unaffected.
	double tc = tEpochSeconds;
	const double loEdge = traj.startTime() + h;
	const double hiEdge = traj.endTime() - h;
	if (loEdge <= hiEdge)
	{
		tc = std::clamp(tEpochSeconds, loEdge, hiEdge);
	}

	const auto tcTicks = timeFromSeconds(tc);
	const CPose3D poseM = traj.poseAt(tcTicks - hTicks);
	const CPose3D pose0 = traj.poseAt(tcTicks);
	const CPose3D poseP = traj.poseAt(tcTicks + hTicks);

	// Kinematics of the IMU frame itself (not of the vehicle frame): its
	// angular velocity in its own axes, and the acceleration of its own
	// origin, which includes the lever-arm terms of an offset mounting.
	const CPose3D sensorM = poseM + sensorPoseOnVehicle;
	const CPose3D sensor0 = pose0 + sensorPoseOnVehicle;
	const CPose3D sensorP = poseP + sensorPoseOnVehicle;

	// --- Angular velocity: exact SO(3) log of the relative rotation,
	// central difference (2nd order accurate, well-defined for any
	// rotation magnitude, unlike an Euler-angle finite difference).
	// `sensorP - sensorM` = sensorM^-1 (+) sensorP, i.e. exactly R_M^T * R_P. ---
	const CPose3D relPose = sensorP - sensorM;
	const auto logRrel = mrpt::poses::Lie::SO<3>::log(relPose.getRotationMatrix());
	const TVector3D trueW(logRrel[0] / (2 * h), logRrel[1] / (2 * h), logRrel[2] / (2 * h));

	// --- Proper (specific) acceleration, body frame at t: finite-difference
	// second derivative of position, minus gravity, rotated into the body
	// frame. A stationary sensor reads +9.81 on local Z, as expected. ---
	const TPoint3D pM = sensorM.translation();
	const TPoint3D p0 = sensor0.translation();
	const TPoint3D pP = sensorP.translation();
	const TVector3D coordAccGlobal(
		(pP.x - 2 * p0.x + pM.x) / (h * h), (pP.y - 2 * p0.y + pM.y) / (h * h),
		(pP.z - 2 * p0.z + pM.z) / (h * h));
	const TVector3D g(0.0, 0.0, -world.get_gravity());
	const TVector3D globalAcc = coordAccGlobal - g;

	const auto R0 = sensor0.getRotationMatrix();
	const TVector3D trueAccLocal(
		R0(0, 0) * globalAcc.x + R0(1, 0) * globalAcc.y + R0(2, 0) * globalAcc.z,
		R0(0, 1) * globalAcc.x + R0(1, 1) * globalAcc.y + R0(2, 1) * globalAcc.z,
		R0(0, 2) * globalAcc.x + R0(1, 2) * globalAcc.y + R0(2, 2) * globalAcc.z);

	const TVector3D w = noiseModel.applyGyroscope(trueW, sampleDt);
	const TVector3D linAccLocal = noiseModel.applyAccelerometer(trueAccLocal, sampleDt);

	auto obs = mrpt::obs::CObservationIMU::Create();
	obs->timestamp = timeFromSeconds(tEpochSeconds);
	obs->sensorLabel = sensorLabel;

	obs->set(mrpt::obs::IMU_WX, w.x);
	obs->set(mrpt::obs::IMU_WY, w.y);
	obs->set(mrpt::obs::IMU_WZ, w.z);
	obs->set(mrpt::obs::IMU_X_ACC, linAccLocal.x);
	obs->set(mrpt::obs::IMU_Y_ACC, linAccLocal.y);
	obs->set(mrpt::obs::IMU_Z_ACC, linAccLocal.z);

	if (measureOrientation)
	{
		const double w2enu = world.georeferenceOptions().world_to_enu_rotation;

		mrpt::math::CVectorFixed<double, 3> oriNoiseVec;
		if (orientationStdNoise > 0)
		{
			std::normal_distribution<double> noiseDist(0.0, orientationStdNoise);
			oriNoiseVec[0] = noiseDist(rng);
			oriNoiseVec[1] = noiseDist(rng);
			oriNoiseVec[2] = noiseDist(rng);
		}
		else
		{
			oriNoiseVec.setZero();
		}
		const auto oriNoiseRot = mrpt::poses::Lie::SO<3>::exp(oriNoiseVec);

		const CPose3D sensorPoseEnu =
			CPose3D::FromYawPitchRoll(w2enu, 0.0, 0.0) + sensor0 +
			CPose3D::FromRotationAndTranslation(oriNoiseRot, TVector3D(0, 0, 0));

		mrpt::math::CQuaternionDouble q;
		sensorPoseEnu.getAsQuaternion(q);

		obs->set(mrpt::obs::IMU_ORI_QUAT_W, q.r());
		obs->set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
		obs->set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
		obs->set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());
	}

	return obs;
}
