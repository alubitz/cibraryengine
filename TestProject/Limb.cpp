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
			joints.emplace_back(joints_[i]);

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
			RigidBody* body = rigid_bodies[i + 1];

			JointEntry& joint = joints[i];
			JointConstraint* jc = joint.constraint;

			Quaternion a_ori = jc->obj_a->GetOrientation();
			Quaternion b_ori = jc->obj_b->GetOrientation();

			Quaternion a_to_b = Quaternion::Reverse(a_ori) * b_ori;

			const Mat3& oriented_axes = joint.oriented_axes = jc->axes * a_ori.ToMat3();

			Vec3 error = joint.desired_pyr - (oriented_axes.Transpose() * a_to_b.ToPYR());
			Debug(((stringstream&)(stringstream() << "error = (" << error.x << ", " << error.y << ", " << error.z << ")" << endl)).str());
			error.y = error.z = 0.0f;

			Vec3 derivative = (error - joint.old_error) * physics_rate;
			joint.error_integral += error * timestep;

			joint.old_error = error;

			joint.alpha = error * 19.0f + joint.error_integral * 10.0f + derivative * 3.0f;
		}
		Debug("\n");

		for(unsigned int i = 0; i < num_joints; ++i)
		{
			JointConstraint* jc = joints[i].constraint;

			Mat3 net_moi = (Mat3(jc->obj_a->GetTransformedMassInfo().moi) + Mat3(jc->obj_a->GetTransformedMassInfo().moi));
			const Mat3& oriented_axes = joints[i].oriented_axes;

			Vec3& alpha = joints[i].alpha;

			float max_mag = 25.0f;
			float mag = alpha.ComputeMagnitude();
			if(mag > max_mag)
				alpha *= max_mag / mag;

			alpha = oriented_axes * alpha;

			jc->motor_torque = oriented_axes.Transpose() * net_moi * alpha * physics_rate;
		}
	}
}
