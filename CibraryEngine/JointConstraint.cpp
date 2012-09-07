#include "StdAfx.h"
#include "JointConstraint.h"

#include "RigidBody.h"

#include "Random3D.h"

namespace CibraryEngine
{
	/*
	 * JointConstraint methods
	 */
	JointConstraint::JointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& pos, const Mat3& axes, const Vec3& min_extents, const Vec3& max_extents, const Vec3& angular_damp) :
		PhysicsConstraint(ibody, jbody),
		desired_ori(Quaternion::Identity()),
		pos(pos),
		axes(axes),
		min_extents(min_extents),
		max_extents(max_extents),
		angular_damp(angular_damp),
		enable_motor(false)
	{
	}

	void JointConstraint::DoConstraintAction(vector<RigidBody*>& wakeup_list)
	{
		static const float angular_vel_coeff =	1.0f;
		static const float dv_coeff =			1.0f;

		const float inv_foresight =				inv_timestep;
		const float foresight =					timestep;

		bool wakeup = false;


		// linear stuff
		Vec3 current_dv = obj_b->GetLocalVelocity(apply_pos) - obj_a->GetLocalVelocity(apply_pos);

		Vec3 dv = desired_dv - current_dv;
		if(float magsq = dv.ComputeMagnitudeSquared())
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

		Vec3 alpha;												// delta-angular-velocity

		// torque to make the joint conform to a pose
		if(enable_motor)
			alpha = (desired_av - current_av) * -angular_vel_coeff;
#if 1
		// enforce joint rotation limits
		Vec3 proposed_av = current_av - alpha;
		Quaternion proposed_ori = a_to_b * Quaternion::FromPYR(proposed_av.x * foresight, proposed_av.y * foresight, proposed_av.z * foresight);
		Vec3 proposed_pyr = oriented_axes * -proposed_ori.ToPYR();

		bool any_changes = false;
		if(proposed_pyr.x < min_extents.x)		{ proposed_pyr.x = min_extents.x; any_changes = true; }
		else if(proposed_pyr.x > max_extents.x)	{ proposed_pyr.x = max_extents.x; any_changes = true; }
		if(proposed_pyr.y < min_extents.y)		{ proposed_pyr.y = min_extents.y; any_changes = true; }
		else if(proposed_pyr.y > max_extents.y)	{ proposed_pyr.y = max_extents.y; any_changes = true; }
		if(proposed_pyr.z < min_extents.z)		{ proposed_pyr.z = min_extents.z; any_changes = true; }
		else if(proposed_pyr.z > max_extents.z)	{ proposed_pyr.z = max_extents.z; any_changes = true; }

		if(any_changes)
		{
			// at least one rotation limit was violated, so we must recompute alpha
			Quaternion actual_ori = Quaternion::FromPYR(reverse_oriented_axes * -proposed_pyr);
			Vec3 actual_av = (b_to_a * actual_ori).ToPYR() * inv_foresight;

			alpha = current_av - actual_av;
		}
#endif

		// apply angular velocity changes
		if(alpha.ComputeMagnitudeSquared())
		{
			Vec3 angular_impulse = moi * alpha;

			obj_a->ApplyAngularImpulse(angular_impulse);
			obj_b->ApplyAngularImpulse(-angular_impulse);

			wakeup = true;
		}


		if(wakeup)
		{
			wakeup_list.push_back(obj_a);
			wakeup_list.push_back(obj_b);
		}
	}

	void JointConstraint::DoUpdateAction(float timestep_)
	{
		timestep = timestep_;
		inv_timestep = 1.0f / timestep;

		const float pyr_coeff =					inv_timestep;
		const float spring_coeff =				inv_timestep;

		Quaternion a_ori = obj_a->GetOrientation();
		Quaternion b_ori = obj_b->GetOrientation();

		a_to_b = Quaternion::Reverse(a_ori) * b_ori;
		b_to_a = Quaternion::Reverse(a_to_b);


		// torque to make the joint conform to a pose
		if(enable_motor)
			desired_av = (Quaternion::Reverse(a_ori) * desired_ori * b_ori).ToPYR() * (-pyr_coeff);

		moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());

		oriented_axes = axes.Transpose() * a_ori.ToMat3();
		reverse_oriented_axes = oriented_axes.Transpose();


		// force to keep the two halves of the joint together
		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos);

		apply_pos = (a_pos + b_pos) * 0.5f;
		desired_dv = (b_pos - a_pos) * -spring_coeff;
	}
}
