#include "StdAfx.h"
#include "JointConstraint.h"

#include "RigidBody.h"

#include "Random3D.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * JointConstraint methods
	 */
	JointConstraint::JointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& pos, const Mat3& axes, const Vec3& min_extents, const Vec3& max_extents, const Vec3& angular_damp) :
		PhysicsConstraint(ibody, jbody),
		desired_ori(Quaternion::Identity()),
		inv_desired(Quaternion::Identity()),
		pos(pos),
		axes(axes),
		min_extents(min_extents),
		max_extents(max_extents),
		angular_damp(angular_damp),
		enable_motor(true)
	{
	}

	void JointConstraint::DoConstraintAction(vector<RigidBody*>& wakeup_list)
	{
		static const float angular_vel_coeff =	1.0f;
		static const float dv_coeff =			1.0f;

		bool wakeup = false;

		if(enable_motor)
		{
			// torque to make the joint conform to a pose
			Vec3 current_av = obj_b->GetAngularVelocity() - obj_a->GetAngularVelocity();

			Vec3 alpha = (desired_av - current_av) * -angular_vel_coeff;
			if(alpha.ComputeMagnitudeSquared() > 0.0f)
			{
				Vec3 angular_impulse = moi * alpha;

				obj_a->ApplyAngularImpulse(angular_impulse);
				obj_b->ApplyAngularImpulse(-angular_impulse);

				wakeup = true;
			}
		}

		// force to keep the two halves of the joint together
		Vec3 current_dv = obj_b->GetLocalVelocity(apply_pos) - obj_a->GetLocalVelocity(apply_pos);

		Vec3 dv = desired_dv - current_dv;
		float mag = dv.ComputeMagnitude();
		if(mag > 0)
		{
			dv /= mag;

			Vec3 impulse = dv * (-mag * dv_coeff * PhysicsWorld::GetUseMass(obj_a, obj_b, apply_pos, dv));
			obj_a->ApplyWorldImpulse(impulse, apply_pos);
			obj_b->ApplyWorldImpulse(-impulse, apply_pos);

			wakeup = true;
		}

		if(wakeup)
		{
			wakeup_list.push_back(obj_a);
			wakeup_list.push_back(obj_b);
		}
	}

	void JointConstraint::DoUpdateAction(float timestep)
	{
		static const float pyr_coeff =			60.0f;			// based on the assumption of physics running at 60hz (maybe requires changing?)
		static const float spring_coeff =		60.0f;

		if(enable_motor)
		{
			// torque to make the joint conform to a pose
			Quaternion a_ori = inv_desired * obj_a->GetOrientation();
			Quaternion b_ori = obj_b->GetOrientation();
			Quaternion a_to_b = Quaternion::Invert(a_ori) * b_ori;

			Vec3 pyr = -a_to_b.ToPYR();
			desired_av = pyr * pyr_coeff;

			moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());
		}

		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos);

		// force to keep the two halves of the joint together
		apply_pos = (a_pos + b_pos) * 0.5f;
		desired_dv = (b_pos - a_pos) * -spring_coeff;
	}

	void JointConstraint::SetDesiredOrientation(const Quaternion& ori)
	{
		if(ori != desired_ori)
		{
			desired_ori = ori;
			inv_desired = Quaternion::Invert(desired_ori);
		}
	}
	Quaternion JointConstraint::GetDesiredOrientation() const { return desired_ori; }
}
