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

			Vec3 surface_normal;

			Quaternion desired_ori;

			float angular_coeff;

			bool broken;
			bool is_in_world;


			// cached stuff

			Vec3 apply_pos;
			Vec3 r1, r2;
			Vec3 desired_dv;

			Vec3 desired_av;
			Vec3 cur_normal;

			float inv_amass, inv_bmass;
			Mat3 rlv_to_impulse, impulse_to_arot, impulse_to_brot;
			Mat3 alpha_to_arot, alpha_to_brot;

			PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& foot_pos, const Vec3& surface_pos, const Vec3& surface_normal);
			PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& foot_pos, const Vec3& surface_pos, const Vec3& surface_normal, const Quaternion& relative_ori, float angular_coeff);

			bool DoConstraintAction();
			void DoUpdateAction(float timestep);

			void OnObjectRemoved(RigidBody* object);
	};
}
