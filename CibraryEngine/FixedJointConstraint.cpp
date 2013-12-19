#include "StdAfx.h"
#include "FixedJointConstraint.h"

#include "RigidBody.h"

namespace CibraryEngine
{
	/*
	 * FixedJointConstraint methods
	 */
	FixedJointConstraint::FixedJointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& a_pos, const Vec3& b_pos, const Quaternion& desired_ori) :
		PhysicsConstraint(ibody, jbody),
		a_pos(a_pos),
		b_pos(b_pos),
		desired_ori(desired_ori)
	{
	}

	bool FixedJointConstraint::DoConstraintAction()
	{
		bool wakeup = false;

		Vec3 avel = obj_a->GetLinearVelocity();
		Vec3 bvel = obj_b->GetLinearVelocity();
		Vec3 arot = obj_a->GetAngularVelocity();
		Vec3 brot = obj_b->GetAngularVelocity();

		// linear stuff
		Vec3 current_dv = bvel - avel + Vec3::Cross(r2, brot) - Vec3::Cross(r1, arot);
		Vec3 dv = desired_dv - current_dv;
		if(float magsq = dv.ComputeMagnitudeSquared())
		{
			Vec3 impulse = rlv_to_impulse * dv;

			// apply impulse
			avel += impulse * inv_amass;
			bvel -= impulse * inv_bmass;
			arot += impulse_to_arot * impulse;
			brot -= impulse_to_brot * impulse;

			wakeup = true;
		}

		// angular stuff
		Vec3 alpha = (brot - arot) - desired_av;
		if(float magsq = alpha.ComputeMagnitudeSquared())
		{
			arot += alpha_to_arot * alpha;
			brot -= alpha_to_brot * alpha;

			wakeup = true;
		}

		if(wakeup)
		{
			obj_a->SetLinearVelocity(avel);
			obj_b->SetLinearVelocity(bvel);
			obj_a->SetAngularVelocity(arot);
			obj_b->SetAngularVelocity(brot);
			
			return true;
		}
		else
			return false;
	}

	void FixedJointConstraint::DoUpdateAction(float timestep)
	{
		float inv_timestep = 1.0f / timestep;

		Vec3 p1 = obj_a->GetTransformationMatrix().TransformVec3_1(a_pos);
		Vec3 p2 = obj_b->GetTransformationMatrix().TransformVec3_1(b_pos);

		apply_pos = (p1 + p2) * 0.5f;
		desired_dv = (p2 - p1) * -inv_timestep;

		r1 = apply_pos - obj_a->GetCenterOfMass();
		r2 = apply_pos - obj_b->GetCenterOfMass();

		if(inv_amass = obj_a->GetMass()) { inv_amass = 1.0f / inv_amass; }
		if(inv_bmass = obj_b->GetMass()) { inv_bmass = 1.0f / inv_bmass; }

		// computing rlv-to-impulse matrix
		Mat3 xr1(
				0,	  r1.z,	  -r1.y,
			-r1.z,	     0,	   r1.x,
			 r1.y,	 -r1.x,	      0		);
		Mat3 xr2(
				0,	  r2.z,	  -r2.y,
			-r2.z,	     0,	   r2.x,
			 r2.y,	 -r2.x,	      0		);

		float invmasses = -(inv_amass + inv_bmass);
		Mat3 a_invmoi = obj_a->GetInvMoI(), b_invmoi = obj_b->GetInvMoI();

		Mat3 impulse_to_rlv = Mat3(invmasses, 0, 0, 0, invmasses, 0, 0, 0, invmasses)
			+ xr1 * a_invmoi * xr1
			+ xr2 * b_invmoi * xr2;

		rlv_to_impulse = Mat3::Invert(impulse_to_rlv);
		impulse_to_arot = a_invmoi * xr1;
		impulse_to_brot = b_invmoi * xr2;

		Mat3 net_moi = Mat3::Invert(a_invmoi + b_invmoi);
		alpha_to_arot = a_invmoi * net_moi;
		alpha_to_brot = b_invmoi * net_moi;

		Quaternion a_ori = obj_a->GetOrientation(), b_ori = obj_b->GetOrientation();
		desired_av = (a_ori * Quaternion::Reverse(desired_ori) * Quaternion::Reverse(b_ori)).ToPYR() * -inv_timestep;
	}
}
