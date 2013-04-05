#include "StdAfx.h"

#include "PlacedFootConstraint.h"

namespace Test
{
	using namespace CibraryEngine;

	/*
	 * PlacedFootConstraint methods
	 */
	PlacedFootConstraint::PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& pos, float angular_coeff) :
		PhysicsConstraint(foot, surface),
		a_pos(foot->GetInvTransform().TransformVec3_1(pos)),
		b_pos(surface->GetInvTransform().TransformVec3_1(pos)),
		desired_ori(Quaternion::Reverse(foot->GetOrientation()) * surface->GetOrientation()),
		angular_coeff(angular_coeff)
	{
	}

	bool PlacedFootConstraint::DoConstraintAction()
	{
		bool wakeup = false;

		Vec3 avel = obj_a->GetLinearVelocity(), bvel = obj_b->GetLinearVelocity(), arot = obj_a->GetAngularVelocity(), brot = obj_b->GetAngularVelocity();
		Vec3 current_dv = bvel - avel + Vec3::Cross(r2, brot) - Vec3::Cross(r1, arot);

		Vec3 dv = desired_dv - current_dv;
		float magsq = dv.ComputeMagnitudeSquared();
		if(magsq > 0.0f)
		{
			Vec3 impulse = rlv_to_impulse * dv;

			// apply impulse
			avel += impulse * inv_amass;
			arot += impulse_to_arot * impulse;
			bvel -= impulse * inv_bmass;
			brot -= impulse_to_brot * impulse;

			wakeup = true;
		}

		// angular stuff
		if(angular_coeff > 0.0f)
		{
			Vec3 alpha = (brot - arot) - desired_av;
			magsq = alpha.ComputeMagnitudeSquared();
			if(magsq > 0.0f)
			{
				alpha *= angular_coeff;

				arot += alpha_to_obja * alpha;
				brot -= alpha_to_objb * alpha;

				wakeup = true;
			}
		}

		if(wakeup)
		{
			obj_a->SetLinearVelocity(avel);
			obj_a->SetAngularVelocity(arot);

			obj_b->SetLinearVelocity(bvel);
			obj_b->SetAngularVelocity(brot);
			
			return true;
		}
		else
			return false;
	}

	void PlacedFootConstraint::DoUpdateAction(float timestep_)
	{
		timestep = timestep_;
		inv_timestep = 1.0f / timestep;

		Vec3 p1 = obj_a->GetTransformationMatrix().TransformVec3_1(a_pos);
		Vec3 p2 = obj_b->GetTransformationMatrix().TransformVec3_1(b_pos);

		apply_pos = (p1 + p2) * 0.5f;
		desired_dv = (p2 - p1) * -(1.0f * inv_timestep);

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

		// angular stuff
		if(angular_coeff > 0.0f)
		{
			Mat3 net_moi = Mat3::Invert(a_invmoi + b_invmoi);
			alpha_to_obja = a_invmoi * net_moi;
			alpha_to_objb = b_invmoi * net_moi;

			Quaternion a_to_b = Quaternion::Reverse(obj_a->GetOrientation()) * obj_b->GetOrientation();
			desired_av = (Quaternion::Reverse(desired_ori) * a_to_b).ToPYR() * -inv_timestep;
		}
	}
}
