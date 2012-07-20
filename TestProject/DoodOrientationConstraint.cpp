#include "StdAfx.h"
#include "DoodOrientationConstraint.h"

#include "Dood.h"

namespace Test
{
	DoodOrientationConstraint::DoodOrientationConstraint(Dood* dood) : PhysicsConstraint(dood->root_rigid_body, NULL), dood(dood), desired_ori(Quaternion::Identity()) { }

	void DoodOrientationConstraint::DoConstraintAction(vector<RigidBody*>& wakeup_list)
	{
		// motor constants
		static const float pyr_coeff =			30.0f;
		static const float angular_vel_coeff =	1.0f;

		Quaternion a_ori = obj_a->GetOrientation();
		Quaternion b_ori = Quaternion::Invert(desired_ori);
		Quaternion a_to_b = Quaternion::Invert(a_ori) * b_ori;

		Vec3 pyr = -a_to_b.ToPYR();
		Vec3 desired_av = pyr * pyr_coeff;
		Vec3 current_av = -obj_a->GetAngularVelocity();

		Vec3 alpha = (desired_av - current_av) * -angular_vel_coeff;
		if(alpha.ComputeMagnitudeSquared() > 0.0f)
		{
			Mat3 moi = Mat3::Invert(obj_a->GetInvMoI());
			Vec3 angular_impulse = moi * alpha;

			obj_a->ApplyAngularImpulse(angular_impulse);

			wakeup_list.push_back(obj_a);
		}
	}
}
