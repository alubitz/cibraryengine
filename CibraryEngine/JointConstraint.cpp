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
		enable_motor(true),
		orient_absolute(false)
	{
		this->min_extents = Vec3(-0.01f, -0.01f, -0.01f);
		this->max_extents = Vec3(0.01f, 0.01f, 0.01f);
	}

	void JointConstraint::DoConstraintAction(vector<RigidBody*>& wakeup_list)
	{
		static const float angular_vel_coeff =	1.0f;
		static const float dv_coeff =			1.0f;

		bool wakeup = false;

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


		// torque to make the joint conform to a pose
		Vec3 alpha;

		// TODO: instead of enforcing joint limits AFTER the motor does its thing, don't let the motor try to violate joint limits
		if(enable_motor)
		{
			Vec3 current_av = orient_absolute ? obj_b->GetAngularVelocity() : obj_b->GetAngularVelocity() - obj_a->GetAngularVelocity();

			alpha = (desired_av - current_av) * -angular_vel_coeff;
		}


		// enforce joint rotation limits
		if(rot_limits)
		{
			Vec3 current_av = (obj_b->GetAngularVelocity() - obj_a->GetAngularVelocity()) - alpha;

			const unsigned int oxo1[] = { 0x01, 0x04, 0x10 };
			const unsigned int oxo3[] = { 0x03, 0x0C, 0x18 };

			for(int i = 0; i < 3; ++i)			// go through limits for each axis
				if(rot_limits & oxo3[i])
				{
					const Vec3& axis = oriented_axes[i];
					float dot = Vec3::Dot(current_av, axis);

					if((rot_limits & oxo1[i]) ? dot < 0.0f : dot > 0.0f)
						alpha += axis * dot;
				}
		}


		// apply angular velocity changes
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
			wakeup_list.push_back(obj_b);
		}
	}

	void JointConstraint::DoUpdateAction(float timestep)
	{
		static const float pyr_coeff =			360.0f;			// based on the assumption of physics running at 360hz (maybe requires changing?)
		static const float spring_coeff =		360.0f;

		// torque to make the joint conform to a pose
		if(enable_motor)
		{
			if(orient_absolute)
				desired_av = (inv_desired * obj_b->GetOrientation()).ToPYR() * (-pyr_coeff);
			else
			{
				Quaternion a_ori = inv_desired * obj_a->GetOrientation();
				Quaternion b_ori = obj_b->GetOrientation();
				Quaternion a_to_b = Quaternion::Reverse(a_ori) * b_ori;
				Vec3 pyr = -a_to_b.ToPYR();

				desired_av = pyr * pyr_coeff;
			}

			moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());
		}


		// enforce joint rotation limits
		Quaternion a_to_b = Quaternion::Reverse(obj_a->GetOrientation()) * obj_b->GetOrientation();

		Mat3 oriented_axes = obj_a->GetOrientation().ToMat3() * axes;
		this->oriented_axes[0] = Vec3(oriented_axes[0], oriented_axes[1], oriented_axes[2]);
		this->oriented_axes[1] = Vec3(oriented_axes[3], oriented_axes[4], oriented_axes[5]);
		this->oriented_axes[2] = Vec3(oriented_axes[6], oriented_axes[7], oriented_axes[8]);
		
		Vec3 pyr = oriented_axes * a_to_b.ToPYR();

		rot_limits = 0;

		if(pyr.x < min_extents.x)		{ rot_limits |= 0x01; }
		else if(pyr.x > max_extents.x)	{ rot_limits |= 0x02; }

		if(pyr.y < min_extents.y)		{ rot_limits |= 0x04; }
		else if(pyr.y > max_extents.y)	{ rot_limits |= 0x08; }

		if(pyr.z < min_extents.z)		{ rot_limits |= 0x10; }
		else if(pyr.z > max_extents.z)	{ rot_limits |= 0x20; }


		// force to keep the two halves of the joint together
		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos);

		apply_pos = (a_pos + b_pos) * 0.5f;
		desired_dv = (b_pos - a_pos) * -spring_coeff;
	}

	void JointConstraint::SetDesiredOrientation(const Quaternion& ori)
	{
		if(ori != desired_ori)
		{
			desired_ori = ori;
			inv_desired = Quaternion::Reverse(desired_ori);
		}
	}
	Quaternion JointConstraint::GetDesiredOrientation() const { return desired_ori; }
}
