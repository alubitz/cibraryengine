#include "StdAfx.h"
#include "Limb.h"

#include "LimbAction.h"

namespace Test
{
	/*
	 * Limb methods
	 */
	Limb::Limb() : joints(), rigid_bodies(), action(NULL) { }

	Limb::~Limb() { if(action) { delete action; action = NULL; } }

	void Limb::Update(float timestep)
	{
		const float physics_rate = 1.0f / timestep;

		if(action != NULL)
			action->Update(timestep);

		bone_torques.resize(joints.size());			// this operation should be effectively free after the first time

		for(unsigned int i = 0; i < joints.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i + 1];

			const Quaternion& desired_ori = joints[i].desired_ori;

			// TODO: deal with exceedingly large angles in a manner that doesn't produce wild flailing

			Quaternion offness = Quaternion::Reverse(body->GetOrientation()) * desired_ori;
			Vec3 desired_av = offness.ToPYR() * physics_rate;

			Vec3 alpha = (desired_av - body->GetAngularVelocity()) * physics_rate;
			bone_torques[i] = Mat3(body->GetTransformedMassInfo().moi) * alpha;
		}

		Vec3 total_torque;
		for(unsigned int i = joints.size() - 1; i < joints.size(); --i)			// this loop condition becomes false when the unsigned int decrements past zero!
		{
			JointConstraint* jc = joints[i].constraint;

			Mat3 oriented_axes(jc->axes * rigid_bodies[i]->GetOrientation().ToMat3());

			total_torque += bone_torques[i];
			jc->motor_torque = oriented_axes.Transpose() * total_torque;
		}

		applied_torque = total_torque;
	}
}
