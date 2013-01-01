#include "StdAfx.h"
#include "RobotArm.h"

#include "Limb.h"

#include "LAClawTest.h"
#include "LAStep.h"

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

		if(is_valid)				// there are some conditions where the Dood may become invalid after a call to Dood::Spawned
		{
			JointConstraint* use_joints[] =
			{
				(JointConstraint*)constraints[0],
				(JointConstraint*)constraints[1],
				(JointConstraint*)constraints[2]
			};

			RigidBody* use_rigid_bodies[] =
			{
				rigid_bodies[0],
				rigid_bodies[1],
				rigid_bodies[2],
				rigid_bodies[3]
			};

			limb = new Limb(use_joints, use_rigid_bodies, 3);

			//limb->action = new LAClawTest(limb);
			limb->action = new LAStep(limb);
		}
	}
}
