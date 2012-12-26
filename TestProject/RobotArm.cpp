#include "StdAfx.h"
#include "RobotArm.h"

#include "Limb.h"

#include "LAClawTest.h"

namespace Test
{
	/*
	 * RobotArm methods
	 */
	RobotArm::RobotArm(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(gs, model, mphys, pos, team),
		limb(NULL)
	{
		hp *= 1000.0f;
	}

	void RobotArm::InnerDispose()
	{
		Dood::InnerDispose();

		if(limb) { delete limb; limb = NULL; }
	}

	void RobotArm::PoseToPhysics(float timestep)
	{
		limb->Update(timestep);
	}

	void RobotArm::Spawned()
	{
		Dood::Spawned();

		if(is_valid)
		{
			limb = new Limb();

			limb->joints.reserve(constraints.size());

			for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			{
				JointConstraint* jc = (JointConstraint*)*iter;

				limb->joints.push_back(Limb::JointEntry(jc));
			}

			limb->rigid_bodies.reserve(rigid_bodies.size());

			for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
				limb->rigid_bodies.push_back(*iter);

			limb->action = new LAClawTest(limb);
		}
	}
}
