#include "StdAfx.h"
#include "Limb.h"

#include "LimbAction.h"

namespace Test
{
	/*
	 * Limb methods
	 */
	Limb::Limb(JointConstraint** joints_, RigidBody** rigid_bodies_, unsigned int count) :
		joints(),
		rigid_bodies(),
		action(NULL)
	{
		joints.reserve(count);
		rigid_bodies.reserve(count + 1);

		for(unsigned int i = 0; i < count; ++i)
			joints.push_back(JointEntry(joints_[i], Vec3(2000.0f, 2000.0f, 100.0f)));

		for(unsigned int i = 0; i < count + 1; ++i)
			rigid_bodies.emplace_back(rigid_bodies_[i]);
	}

	Limb::~Limb() { if(action) { delete action; action = NULL; } }

	void Limb::Update(float timestep)
	{
		const float inv_timestep = 1.0f / timestep;

		if(action != NULL)
			action->Update(timestep);

		unsigned int num_joints = joints.size();

		for(unsigned int i = 0; i < num_joints; ++i)
		{
			JointEntry& joint = joints[i];
			JointConstraint* jc = joint.constraint;

			RigidBody *obj_a = jc->obj_a, *obj_b = jc->obj_b;

			Quaternion a_ori = obj_a->GetOrientation(), b_ori = obj_b->GetOrientation();
			Quaternion a_to_b = Quaternion::Reverse(a_ori) * b_ori;

			const Mat3& oriented_axes = joint.oriented_axes = jc->axes * a_ori.ToMat3();

			Vec3 error = oriented_axes * a_to_b.ToPYR() + joint.desired_pyr;

			// TODO: do this in a less hackish manner
			if(i != 0)
				error.y = error.z = 0.0f;			// joints other than the shoulder joint can only rotate on their primary axis

			Vec3 derivative = (error - joint.old_error) * inv_timestep;
			joint.error_integral += error * timestep;
			joint.old_error = error;

			const Vec3& pid_coeffs = joint.pid_coeffs;
			Vec3 pid_out = error * pid_coeffs.x + joint.error_integral * pid_coeffs.y + derivative * pid_coeffs.z;

			joint.bone_torque = Mat3(obj_b->GetTransformedMassInfo().moi) * oriented_axes.Transpose() * pid_out;
		}

		float max_mags[] = { 200, 200, 200 };

		Vec3 use_torque;
		for(unsigned int i = num_joints - 1; i < num_joints; --i)			// NOTE: this loop stops when the unsigned int decrements past zero
		{
			JointEntry& joint = joints[i];
			JointConstraint* jc = joint.constraint;

			const Mat3& oriented_axes = joint.oriented_axes;

			use_torque += joint.bone_torque;

			float max_mag = max_mags[i];
			float mag = use_torque.ComputeMagnitude();
			if(mag > max_mag)
				use_torque *= max_mag / mag;

			Vec3 goal_torque = oriented_axes * use_torque;
			float a_coeff = 0.75f, b_coeff = 1.0f - a_coeff;

			jc->motor_torque = jc->motor_torque * a_coeff + goal_torque * b_coeff;
		}

		applied_torque = use_torque;
	}
}
