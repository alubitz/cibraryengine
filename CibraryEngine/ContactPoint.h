#pragma once

#include "StdAfx.h"

#include "Vector.h"
#include "Matrix.h"

#include "Physics.h"

namespace CibraryEngine
{
	/** A point of contact between two physics objects */
	struct ContactPoint : public PhysicsConstraint
	{
		Vec3 pos;
		Vec3 normal;

		// cached values computed in DoUpdateAction
		float restitution_coeff, fric_coeff;

		Vec3 r1, r2;
		Mat3 rlv_to_impulse;
		Mat3 impulse_to_arot, impulse_to_brot;

		float bounce_threshold;

		ContactPoint() { }
		ContactPoint(RigidBody* obj_a, RigidBody* obj_b) : PhysicsConstraint(obj_a, obj_b) { }


		Vec3 GetRelativeLocalVelocity() const;
		void ApplyImpulse(const Vec3& impulse) const;

		bool DoConstraintAction();
		void DoUpdateAction(float timestep);

		bool DoCollisionResponse() const;
	};
}
