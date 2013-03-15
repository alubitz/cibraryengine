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

		bool cache_valid;
		// cached values (must be computed if cache_valid is false)
		float restitution_coeff, fric_coeff;

		Vec3 r1, r2;
		Mat3 rlv_to_impulse;

		// more cached values, but these are computed in DoUpdateAction instead of BuildCache
		float timestep;
		float bounce_threshold;

		ContactPoint() : cache_valid(false) { }
		ContactPoint(RigidBody* obj_a, RigidBody* obj_b) : PhysicsConstraint(obj_a, obj_b), cache_valid(false) { }
		~ContactPoint() { }


		void BuildCache();

		Vec3 GetRelativeLocalVelocity() const;
		void ApplyImpulse(const Vec3& impulse) const;

		bool DoConstraintAction();
		void DoUpdateAction(float timestep);

		bool DoCollisionResponse() const;
	};
}
