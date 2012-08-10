#include "StdAfx.h"
#include "PlacedFootConstraint.h"

#include "RigidBody.h"

namespace CibraryEngine
{
	/*
	 * PlacedFootConstraint methods
	 */
	PlacedFootConstraint::PlacedFootConstraint(RigidBody* foot, RigidBody* base, const Vec3& pos) :
		PhysicsConstraint(foot, base),
		pos_in_a(foot->GetInvTransform().TransformVec3_1(pos)),
		pos_in_b(base->GetInvTransform().TransformVec3_1(pos)),
		ori(Quaternion::Reverse(foot->GetOrientation()) * base->GetOrientation())
	{
	}

	void PlacedFootConstraint::DoConstraintAction(vector<RigidBody*>& wakeup_list)
	{
		static const float angular_vel_coeff =	1.0f;
		static const float dv_coeff =			1.0f;

		bool wakeup = false;

		// TODO: detect a situation that should break the constraint, and act accordingly

		// linear stuff
		Vec3 current_dv = obj_b->GetLocalVelocity(apply_pos) - obj_a->GetLocalVelocity(apply_pos);

		Vec3 dv = desired_dv - current_dv;
		float magsq = dv.ComputeMagnitudeSquared();
		if(magsq > 0.0f)
		{
			Vec3 impulse = dv * (-dv_coeff * PhysicsWorld::GetUseMass(obj_a, obj_b, apply_pos, dv / sqrtf(magsq)));
			obj_a->ApplyWorldImpulse(impulse, apply_pos);
			obj_b->ApplyWorldImpulse(-impulse, apply_pos);

			wakeup = true;
		}

		// angular stuff
		Vec3 a_avel = obj_a->GetAngularVelocity();
		Vec3 b_avel = obj_b->GetAngularVelocity();
		Vec3 current_av = b_avel - a_avel;

		Vec3 alpha = (desired_av - current_av) * -angular_vel_coeff;												// delta-angular-velocity

		if(alpha.ComputeMagnitudeSquared() > 0.0f)
		{
			Vec3 angular_impulse = moi * alpha;

			obj_a->ApplyAngularImpulse(angular_impulse);
			obj_b->ApplyAngularImpulse(-angular_impulse);

			wakeup = true;
		}


		if(wakeup)
		{
			wakeup_list.push_back(obj_a);

			if(obj_b->MergesSubgraphs())
				wakeup_list.push_back(obj_b);
		}
	}

	void PlacedFootConstraint::DoUpdateAction(float timestep)
	{
		float inv_timestep = 1.0f / timestep;

		const float pyr_coeff =			inv_timestep;
		const float spring_coeff =		inv_timestep;

		Quaternion a_ori = obj_a->GetOrientation();
		Quaternion b_ori = obj_b->GetOrientation();

		// torque to make the joint stay in the same orientation
		desired_av = (Quaternion::Reverse(a_ori) * ori * b_ori).ToPYR() * (-pyr_coeff);
		moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());

		// force to keep the two objects together
		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos_in_a);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos_in_b);

		apply_pos = (a_pos + b_pos) * 0.5f;
		desired_dv = (b_pos - a_pos) * -spring_coeff;
	}
}
