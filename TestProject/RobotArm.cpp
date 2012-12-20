#include "StdAfx.h"
#include "RobotArm.h"

namespace Test
{
	/*
	 * RobotArm methods
	 */
	RobotArm::RobotArm(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(gs, model, mphys, pos, team),
		joints(),
		claw_timer(0.0f)
	{
		hp *= 1000.0f;
	}

	void RobotArm::PoseToPhysics(float timestep)
	{
		const float physics_rate = 1.0f / timestep;

		claw_timer += timestep;
		joints[joints.size() - 1].desired_ori = Quaternion::FromPYR(Vec3::Normalize(Vec3(-0.0336f, 0.0, 0.1125f), sinf(claw_timer)));

		vector<Vec3> bone_torques(rigid_bodies.size());

		for(unsigned int i = 1; i < rigid_bodies.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i];

			Quaternion desired_ori = joints[i - 1].desired_ori;

			Quaternion offness = Quaternion::Reverse(body->GetOrientation()) * desired_ori;
			Vec3 desired_av = offness.ToPYR() * physics_rate;

			Vec3 alpha = (desired_av - body->GetAngularVelocity()) * physics_rate;
			bone_torques[i] = Mat3(body->GetTransformedMassInfo().moi) * alpha;
		}

		Vec3 total_torque;
		for(unsigned int i = joints.size() - 1; i < joints.size(); --i)
		{
			const Vec3& torque = bone_torques[i + 1];

			total_torque += torque;

			Mat3 oriented_axes = joints[i].constraint->axes * rigid_bodies[i]->GetOrientation().ToMat3();
			joints[i].constraint->motor_torque = oriented_axes.Transpose() * total_torque;
		}
	}

	void RobotArm::Spawned()
	{
		Dood::Spawned();

		if(is_valid)
		{
			joints.reserve(constraints.size());

			for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			{
				JointConstraint* jc = (JointConstraint*)*iter;

				joints.push_back(JointEntry(jc));
			}
		}
	}
}
