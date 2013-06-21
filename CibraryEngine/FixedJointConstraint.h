#pragma once

#include "StdAfx.h"

#include "Physics.h"
#include "Matrix.h"

namespace CibraryEngine
{
	using namespace std;

	class RigidBody;

	class FixedJointConstraint : public PhysicsConstraint
	{
		protected:

			// cached stuff

			Vec3 apply_pos;
			Vec3 r1, r2;
			Vec3 desired_dv, desired_av;

			float inv_amass, inv_bmass;
			Mat3 rlv_to_impulse, impulse_to_arot, impulse_to_brot;
			Mat3 alpha_to_arot, alpha_to_brot;

		public:

			// persistent stuff

			Vec3 a_pos;
			Vec3 b_pos;

			Quaternion desired_ori;

			FixedJointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& a_pos, const Vec3& b_pos, const Quaternion& desired_ori);

			bool DoConstraintAction();
			void DoUpdateAction(float timestep);
	};
}
