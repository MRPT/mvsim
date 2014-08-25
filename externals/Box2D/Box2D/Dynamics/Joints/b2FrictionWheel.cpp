/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2FrictionWheel.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <stdio.h>

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2FrictionWheelDef::Initialize(b2Body* bodyWheel, const b2Vec2& anchor, float32 localWheelAngle)
{
	bodyWheel = bodyWheel;
	localWheelPt = bodyWheel->GetLocalPoint(anchor);
	this->localWheelAngle = localWheelAngle;
}

b2FrictionWheel::b2FrictionWheel(const b2FrictionWheelDef* def)
: b2Joint(def)
{
	m_localWheelPt = def->localWheelPt;

	m_linearImpulse.SetZero();
	m_angularImpulse = 0.0;

	m_maxForce = def->maxForce;
	m_maxTorque = def->maxTorque;
}

void b2FrictionWheel::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_invMassA = m_bodyA->m_invMass;
	m_invIA = m_bodyA->m_invI;

	float32 aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	float32 wA = data.velocities[m_indexA].w;

	b2Rot qA(aA);

	// Compute the effective mass matrix.
	m_rA = b2Mul(qA, m_localWheelPt);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float32 mA = m_invMassA;
	float32 iA = m_invIA;

	b2Mat22 K;
	K.ex.x = mA + iA * m_rA.y * m_rA.y;
	K.ex.y = -iA * m_rA.x * m_rA.y;
	K.ey.x = K.ex.y;
	K.ey.y = mA + iA * m_rA.x * m_rA.x;

	m_linearMass = K.GetInverse();

	m_angularMass = iA;
	if (m_angularMass > 0.0)
	{
		m_angularMass = 1.0f / m_angularMass;
	}

	if (false) //data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_linearImpulse *= data.step.dtRatio;
		m_angularImpulse *= data.step.dtRatio;

		b2Vec2 P(m_linearImpulse.x, m_linearImpulse.y);
		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + m_angularImpulse);
	}
	else
	{
		m_linearImpulse.SetZero();
		m_angularImpulse = 0.0;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
}

void b2FrictionWheel::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[m_indexA].v;
	float32 wA = data.velocities[m_indexA].w;

	float32 mA = m_invMassA;
	float32 iA = m_invIA;

	float32 h = data.step.dt;

	// Solve angular friction
	{
		float32 Cdot = - wA;
		float32 impulse = -m_angularMass * Cdot;

		float32 oldImpulse = m_angularImpulse;
		float32 maxImpulse = h * m_maxTorque;
		m_angularImpulse = b2Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_angularImpulse - oldImpulse;

		wA -= iA * impulse;
	}

	// Solve linear friction
	{
		b2Vec2 Cdot = - vA - b2Cross(wA, m_rA);

		//printf("Cdot: %6.03f,%6.03f\n",Cdot.x,Cdot.y);

		b2Vec2 impulse = -b2Mul(m_linearMass, Cdot);
		b2Vec2 oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		float32 maxImpulse = h * m_maxForce;

		if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
		{
			m_linearImpulse.Normalize();
			m_linearImpulse *= maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		vA -= mA * impulse;
		wA -= iA * b2Cross(m_rA, impulse);
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
}

bool b2FrictionWheel::SolvePositionConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);

	return true;
}

b2Vec2 b2FrictionWheel::GetAnchorA() const
{
	// Use bodyA: the wheel:
	return m_bodyA->GetWorldPoint(m_localWheelPt);
}
b2Vec2 b2FrictionWheel::GetAnchorB() const
{
	// Also use bodyA: the wheel (we don't need the ref to the ground body)
	return m_bodyA->GetWorldPoint(m_localWheelPt);
}

b2Vec2 b2FrictionWheel::GetReactionForce(float32 inv_dt) const
{
	return inv_dt * m_linearImpulse;
}

float32 b2FrictionWheel::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * m_angularImpulse;
}

void b2FrictionWheel::SetMaxForce(float32 force)
{
	b2Assert(b2IsValid(force) && force >= 0.0);
	m_maxForce = force;
}

float32 b2FrictionWheel::GetMaxForce() const
{
	return m_maxForce;
}

void b2FrictionWheel::SetMaxTorque(float32 torque)
{
	b2Assert(b2IsValid(torque) && torque >= 0.0);
	m_maxTorque = torque;
}

float32 b2FrictionWheel::GetMaxTorque() const
{
	return m_maxTorque;
}

void b2FrictionWheel::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;

	b2Log("  b2FrictionWheelDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localWheelPt.Set(%.15lef, %.15lef);\n", m_localWheelPt.x, m_localWheelPt.y);
	b2Log("  jd.maxForce = %.15lef;\n", m_maxForce);
	b2Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
