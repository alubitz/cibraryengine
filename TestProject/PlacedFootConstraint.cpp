#include "StdAfx.h"

#include "PlacedFootConstraint.h"

namespace Test
{
	using namespace CibraryEngine;

	/*
	 * PlacedFootConstraint methods
	 */
	PlacedFootConstraint::PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& a_pos, const Vec3& b_pos, const Vec3& surface_normal) :
		PhysicsConstraint(foot, surface),
		a_pos(a_pos),
		b_pos(b_pos),
		surface_normal(surface_normal),
		angular_coeff(0.0f),
		broken(false),
		is_in_world(false)
	{
	}

	PlacedFootConstraint::PlacedFootConstraint(RigidBody* foot, RigidBody* surface, const Vec3& a_pos, const Vec3& b_pos, const Vec3& surface_normal, const Quaternion& relative_ori, float angular_coeff) :
		PhysicsConstraint(foot, surface),
		a_pos(a_pos),
		b_pos(b_pos),
		surface_normal(surface_normal),
		desired_ori(relative_ori),
		angular_coeff(angular_coeff),
		broken(false),
		is_in_world(false)
	{
	}

	bool PlacedFootConstraint::DoConstraintAction()
	{
		// TODO: improve how tangential and angular breakage work so that they can be re-enabled
		static const float outward_dv_breakage_threshold		= 4.0f;
		static const float friction_bonus_threshold				= 0.1f;
		static const float av_breakage_threshold				= 50.0f;

		assert(is_in_world);

		bool wakeup = false;

		Vec3 avel = obj_a->GetLinearVelocity();
		Vec3 bvel = obj_b->GetLinearVelocity();
		Vec3 arot = obj_a->GetAngularVelocity();
		Vec3 brot = obj_b->GetAngularVelocity();

		Vec3 current_dv = bvel - avel + Vec3::Cross(r2, brot) - Vec3::Cross(r1, arot);
		float dot = Vec3::Dot(current_dv, cur_normal);

		float restitution_impulse_sq = (rlv_to_impulse * cur_normal).ComputeMagnitudeSquared() * dot * dot;

		if(dot < -outward_dv_breakage_threshold)
			broken = true;
		else
		{
			Vec3 full_friction_impulse = rlv_to_impulse * (current_dv - cur_normal * dot);

			if(restitution_impulse_sq > full_friction_impulse.ComputeMagnitudeSquared() + friction_bonus_threshold)
				broken = true;
		}

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
		if(angular_coeff > 0.0f)
		{
			Vec3 alpha = (brot - arot) - desired_av;
			if(float magsq = alpha.ComputeMagnitudeSquared())
			{
				arot += alpha_to_arot * alpha;
				brot -= alpha_to_brot * alpha;

				//if(magsq > av_breakage_threshold)
				//	broken = true;

				wakeup = true;
			}
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

	void PlacedFootConstraint::DoUpdateAction(float timestep)
	{
		float inv_timestep = 1.0f / timestep;

		Vec3 p1 = obj_a->GetTransformationMatrix().TransformVec3_1(a_pos);
		Vec3 p2 = obj_b->GetTransformationMatrix().TransformVec3_1(b_pos);

		cur_normal = obj_b->GetTransformationMatrix().TransformVec3_0(surface_normal);

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
			alpha_to_arot = a_invmoi * net_moi * angular_coeff;
			alpha_to_brot = b_invmoi * net_moi * angular_coeff;

			Quaternion a_to_b = obj_a->GetOrientation() * Quaternion::Reverse(obj_b->GetOrientation());
			desired_av = (a_to_b * Quaternion::Reverse(desired_ori)).ToRVec() * -inv_timestep;
		}
	}

	void PlacedFootConstraint::OnObjectRemoved(RigidBody* object)
	{
		is_in_world = false;
	}
}
