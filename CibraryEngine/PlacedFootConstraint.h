#pragma once
#include "StdAfx.h"

#include "Physics.h"
#include "Matrix.h"

namespace CibraryEngine
{
	class PlacedFootConstraint : public PhysicsConstraint
	{
		protected:

			// some values that are set by DoUpdateAction and which should only be used for the duration of a single physics tick
			Mat3 moi;
			Vec3 apply_pos;
			Vec3 desired_av, desired_dv;

		public:
			
			/** Points defined in each object's local coordinate system; these points will be kept in the same spot */
			Vec3 pos_in_a;
			Vec3 pos_in_b;

			/** Relative orientation of objects */
			Quaternion ori;

			PlacedFootConstraint(RigidBody* foot, RigidBody* base, const Vec3& pos);

			void DoConstraintAction(vector<RigidBody*>& wakeup);
			void DoUpdateAction(float timestep);
	};
}
