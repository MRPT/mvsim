/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mvsim/Suspension.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

using namespace mvsim;

namespace
{
const char* kDofNames[3] = {"heave", "roll", "pitch"};
}

void Suspension::init(const Params& p)
{
	ASSERTMSG_(!p.corners.empty(), "Suspension: the vehicle has no wheels");
	ASSERTMSG_(p.sprungMass > 0, "Suspension: the chassis mass must be positive");
	p_ = p;

	// Corner deflections (up) from the displacement, about the pivot under
	// the center of mass: s_i = z + y_i*roll - x_i*pitch
	const size_t nC = p.corners.size();
	Eigen::MatrixXd J(nC, 3);
	double sumK = 0;
	double minX = 0;
	double maxX = 0;
	double minY = 0;
	double maxY = 0;
	for (size_t i = 0; i < nC; i++)
	{
		const auto& c = p.corners[i];
		const double x = c.pos.x - p.com.x;
		const double y = c.pos.y - p.com.y;
		J(i, 0) = 1.0;
		J(i, 1) = y;
		J(i, 2) = -x;
		sumK += c.stiffness;
		minX = i == 0 ? x : std::min(minX, x);
		maxX = i == 0 ? x : std::max(maxX, x);
		minY = i == 0 ? y : std::min(minY, y);
		maxY = i == 0 ? y : std::max(maxY, y);
	}
	ASSERTMSG_(sumK > 0, "Suspension: the corner springs have no stiffness");
	J_ = mrpt::math::CMatrixDouble(J);

	// Rotations with all corners on their axis have no stiffness: lock them.
	constexpr double minSpread = 1e-3;	// [m]
	active_ = {HEAVE};
	if (maxY - minY > minSpread)
	{
		active_.push_back(ROLL);
	}
	if (maxX - minX > minSpread)
	{
		active_.push_back(PITCH);
	}

	// Full 3-DOF matrices. Inertias are about the pivot (parallel axis
	// theorem), and gravity makes the rotations less stiff when the center of
	// mass is above the pivot.
	const double m = p.sprungMass;
	const double h = p.comHeight;
	Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
	M(0, 0) = m;
	M(1, 1) = p.I_xx + m * h * h;
	M(2, 2) = p.I_yy + m * h * h;
	Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
	for (size_t i = 0; i < nC; i++)
	{
		const Eigen::Vector3d Ji = J.row(i).transpose();
		K += p.corners[i].stiffness * Ji * Ji.transpose();
		C += p.corners[i].damping * Ji * Ji.transpose();
	}
	K(1, 1) -= m * p.gravity * h;
	K(2, 2) -= m * p.gravity * h;

	const int n = static_cast<int>(active_.size());
	Eigen::MatrixXd Mr(n, n);
	Eigen::MatrixXd Kr(n, n);
	Eigen::MatrixXd Cr(n, n);
	for (int r = 0; r < n; r++)
	{
		for (int c = 0; c < n; c++)
		{
			Mr(r, c) = M(active_[r], active_[c]);
			Kr(r, c) = K(active_[r], active_[c]);
			Cr(r, c) = C(active_[r], active_[c]);
		}
	}
	M_ = mrpt::math::CMatrixDouble(Mr);
	K_ = mrpt::math::CMatrixDouble(Kr);
	C_ = mrpt::math::CMatrixDouble(Cr);

	computeModes();
	for (const auto& mode : modes_)
	{
		if (!(mode.frequency_hz > 0))
		{
			THROW_EXCEPTION_FMT(
				"Suspension: the chassis is unstable in %s (the springs are too soft for the "
				"height of its center of mass: increase frequency_hz or the corner stiffness)",
				mode.shape.c_str());
		}
	}

	cachedDt_ = 0;
	delta_.setZero();
	w_.setZero();
	acc_.setZero();
	baseRates_.setZero();
	initialized_ = true;
}

void Suspension::computeModes()
{
	// Undamped modes: K v = w^2 M v, with the modal damping ratio of each
	// one (exact when the damping is proportional to the stiffness):
	const auto& M = M_.asEigen();
	const auto& C = C_.asEigen();
	modes_.clear();
	Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es(K_.asEigen(), M);
	for (int j = 0; j < static_cast<int>(active_.size()); j++)
	{
		const Eigen::VectorXd v = es.eigenvectors().col(j);
		const double w2 = es.eigenvalues()(j);
		Mode mode;
		mode.frequency_hz = w2 > 0 ? std::sqrt(w2) / (2 * M_PI) : 0.0;
		const double vMv = v.dot(M * v);
		mode.damping_ratio = w2 > 0 ? v.dot(C * v) / (2 * std::sqrt(w2) * vMv) : 0.0;

		int dominant = 0;
		for (int k = 1; k < v.size(); k++)
		{
			if (v(k) * v(k) * M(k, k) > v(dominant) * v(dominant) * M(dominant, dominant))
			{
				dominant = k;
			}
		}
		mode.shape = kDofNames[active_[dominant]];
		modes_.push_back(mode);
	}
}

std::string Suspension::modesAsString() const
{
	std::string s;
	for (const auto& mode : modes_)
	{
		s += mrpt::format(
			"%s%s %.2f Hz (zeta=%.2f)", s.empty() ? "" : ", ", mode.shape.c_str(),
			mode.frequency_hz, mode.damping_ratio);
	}
	return s;
}

void Suspension::updateDiscretization(double dt)
{
	// State x=(displacement, absolute velocity), input u=(base rates, forces):
	//  d(displacement)/dt = w - baseRates
	//  M dw/dt = -K displacement - C (w - baseRates) + F
	// Exact solution for constant inputs during dt, from the exponential of
	// the augmented matrix [A B; 0 0]*dt = [Phi Gamma; 0 I].
	const int n = static_cast<int>(active_.size());
	const Eigen::MatrixXd Minv = M_.asEigen().inverse();
	const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
	Eigen::MatrixXd Aug = Eigen::MatrixXd::Zero(4 * n, 4 * n);
	Aug.block(0, n, n, n) = I;
	Aug.block(n, 0, n, n) = -Minv * K_.asEigen();
	Aug.block(n, n, n, n) = -Minv * C_.asEigen();
	Aug.block(0, 2 * n, n, n) = -I;
	Aug.block(n, 2 * n, n, n) = Minv * C_.asEigen();
	Aug.block(n, 3 * n, n, n) = Minv;

	const Eigen::MatrixXd E = (Aug * dt).exp();
	Phi_ = mrpt::math::CMatrixDouble(E.block(0, 0, 2 * n, 2 * n));
	Gamma_ = mrpt::math::CMatrixDouble(E.block(0, 2 * n, 2 * n, 2 * n));
	cachedDt_ = dt;
}

void Suspension::step(double dt, const Vector3& baseRates, const mrpt::math::TVector2D& accel)
{
	ASSERT_(initialized_);
	if (dt <= 0)
	{
		return;
	}
	if (dt != cachedDt_)
	{
		updateDiscretization(dt);
	}

	// Generalized forces: inertia of the chassis, at the height of its center
	// of mass, in the (accelerating) vehicle frame:
	const double mh = p_.sprungMass * p_.comHeight;
	const double F[3] = {0.0, mh * accel.y, -mh * accel.x};

	const int n = static_cast<int>(active_.size());
	Eigen::VectorXd x(2 * n);
	Eigen::VectorXd u(2 * n);
	for (int k = 0; k < n; k++)
	{
		x(k) = delta_[active_[k]];
		x(n + k) = w_[active_[k]];
		u(k) = baseRates[active_[k]];
		u(n + k) = F[active_[k]];
	}
	const Eigen::VectorXd xNew = Phi_.asEigen() * x + Gamma_.asEigen() * u;

	// Locked degrees of freedom follow the base:
	const Vector3 wOld = w_;
	delta_.setZero();
	w_ = baseRates;
	for (int k = 0; k < n; k++)
	{
		delta_[active_[k]] = xNew(k);
		w_[active_[k]] = xNew(n + k);
	}
	for (int dof = 0; dof < 3; dof++)
	{
		acc_[dof] = (w_[dof] - wOld[dof]) / dt;
	}
	baseRates_ = baseRates;
}

Suspension::Vector3 Suspension::relativeVelocity() const
{
	Vector3 v;
	for (int dof = 0; dof < 3; dof++)
	{
		v[dof] = w_[dof] - baseRates_[dof];
	}
	return v;
}

void Suspension::setState(const Vector3& displacement, const Vector3& absoluteVelocity)
{
	delta_ = displacement;
	w_ = absoluteVelocity;
	for (int dof = 0; dof < 3; dof++)
	{
		if (std::find(active_.begin(), active_.end(), dof) == active_.end())
		{
			delta_[dof] = 0;
			w_[dof] = baseRates_[dof];
		}
	}
}

mrpt::poses::CPose3D Suspension::relativePose() const
{
	// Rotation about the pivot point, under the center of mass:
	const auto& c = p_.com;
	return mrpt::poses::CPose3D(c.x, c.y, delta_[HEAVE], 0, delta_[PITCH], delta_[ROLL]) +
		   mrpt::poses::CPose3D(-c.x, -c.y, 0, 0, 0, 0);
}

std::vector<double> Suspension::cornerLoadChanges() const
{
	const Eigen::VectorXd s = J_.asEigen() * delta_.asEigen();
	const Eigen::VectorXd ds = J_.asEigen() * relativeVelocity().asEigen();
	std::vector<double> dN(p_.corners.size());
	for (size_t i = 0; i < dN.size(); i++)
	{
		dN[i] = -(p_.corners[i].stiffness * s(i) + p_.corners[i].damping * ds(i));
	}
	return dN;
}
