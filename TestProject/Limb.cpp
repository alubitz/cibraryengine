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
			joints.push_back(JointEntry(joints_[i], Vec3(22.0f, 9.0f, 2.0f)));

		for(unsigned int i = 0; i < count + 1; ++i)
			rigid_bodies.emplace_back(rigid_bodies_[i]);
	}

	Limb::~Limb() { if(action) { delete action; action = NULL; } }

	void Limb::Update(float timestep)
	{
		const float physics_rate = 1.0f / timestep;

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

			//Debug(((stringstream&)(stringstream() << "error = (" << error.x << ", " << error.y << ", " << error.z << ")" << endl)).str());
			if(i != 0)
				error.y = error.z = 0.0f;			// joints other than the shoulder joint can only rotate on their primary axis

			Vec3 derivative = (error - joint.old_error) * physics_rate;
			joint.error_integral += error * timestep;
			joint.old_error = error;

			const Vec3& pid_coeffs = joint.pid_coeffs;
			Vec3 pid_out = error * pid_coeffs.x + joint.error_integral * pid_coeffs.y + derivative * pid_coeffs.z;

			joint.bone_torque = Mat3(obj_b->GetTransformedMassInfo().moi) * oriented_axes.Transpose() * pid_out;
		}
		//Debug("\n");

		Vec3 use_torque;
		for(unsigned int i = num_joints - 1; i < num_joints; --i)			// NOTE: this loop stops when the unsigned int decrements past zero
		{
			JointEntry& joint = joints[i];
			JointConstraint* jc = joint.constraint;

			const Mat3& oriented_axes = joint.oriented_axes;

			use_torque += joint.bone_torque;

			Mat3 net_moi = Mat3(jc->obj_a->GetTransformedMassInfo().moi) + Mat3(jc->obj_b->GetTransformedMassInfo().moi);
			Mat3 inv_moi = Mat3::Invert(net_moi);

			Vec3 alpha = inv_moi * use_torque;

			float max_mag = 25.0f;
			float mag = alpha.ComputeMagnitude();
			if(mag > max_mag)
			{
				alpha *= max_mag / mag;
				use_torque = net_moi * alpha;
			}

			jc->motor_torque = oriented_axes * use_torque * physics_rate;
		}

		applied_torque = use_torque;
	}
}
