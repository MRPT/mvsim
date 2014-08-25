/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
* Copyright (c) 2014 Jose-Luis Blanco
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_FRICTION_WHEEL_GROUND_H
#define B2_FRICTION_WHEEL_GROUND_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

/// Friction joint definition.
struct b2FrictionWheelDef : public b2JointDef
{
	b2FrictionWheelDef()
	{
		type = e_wheelGroundJoint;
		localWheelPt.SetZero();
		localWheelAngle = 0.0;
		maxForce = 0.0;
		maxTorque = 0.0;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyWheel, const b2Vec2& anchor, float32 localWheelAngle);

	/// The local anchor point relative to bodyWheel's origin.
	b2Vec2  localWheelPt;
	float32 localWheelAngle;

	/// The maximum friction force in N.
	float32 maxForce;

	/// The maximum friction torque in N-m.
	float32 maxTorque;
};

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class b2FrictionWheel : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const;
	b2Vec2 GetAnchorB() const;

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;

	/// The local anchor point relative to bodyWheel's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localWheelPt; }

	/// Set the maximum friction force in N.
	void SetMaxForce(float32 force);

	/// Get the maximum friction force in N.
	float32 GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float32 torque);

	/// Get the maximum friction torque in N*m.
	float32 GetMaxTorque() const;

	/// Dump joint to dmLog
	void Dump();

protected:

	friend class b2Joint;

	b2FrictionWheel(const b2FrictionWheelDef* def);

	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);

	b2Vec2 m_localWheelPt;

	// Solver shared
	b2Vec2 m_linearImpulse;
	float32 m_angularImpulse;
	float32 m_maxForce;
	float32 m_maxTorque;

	// Solver temp
	int32 m_indexA;
	b2Vec2 m_rA;
	float32 m_invMassA;
	float32 m_invIA;
	b2Mat22 m_linearMass;
	float32 m_angularMass;
};

#endif
