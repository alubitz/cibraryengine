#pragma once
#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PlacedFootConstraint : public PhysicsConstraint
	{
		public:

			// persistent stuff
			Vec3 a_pos;
			Vec3 b_pos;

			Quaternion desired_ori;

			float angular_coeff;


			// cached stuff

			Vec3 apply_pos;
			Vec3 r1, r2;
			Vec3 desired_dv;

			Vec3 desired_av;

			float inv_amass, inv_bmass;
			Mat3 rlv_to_impulse, impulse_to_arot, impulse_to_brot;
			Mat3 alpha_to_obja, alpha_to_objb;

			PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& foot_pos, const Vec3& surface_pos);
			PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& foot_pos, const Vec3& surface_pos, const Quaternion& relative_ori, float angular_coeff);

			bool DoConstraintAction();
			void DoUpdateAction(float timestep);
	};
}
