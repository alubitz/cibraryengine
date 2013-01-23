#include "StdAfx.h"
#include "RobotTripod.h"

#include "Limb.h"

#include "LAClawTest.h"
#include "LAStep.h"

namespace Test
{
	/*
	 * RobotTripod methods
	 */
	RobotTripod::RobotTripod(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(gs, model, mphys, pos, team)
	{
		limbs[0] = limbs[1] = limbs[2] = NULL;

		hp *= 1000.0f;
	}

	void RobotTripod::InnerDispose()
	{
		Dood::InnerDispose();

		for(int i = 0; i < 3; ++i)
			if(limbs[i]) { delete limbs[i]; limbs[i] = NULL; }
	}

	void RobotTripod::PoseToPhysics(float timestep)
	{
		for(int i = 0; i < 3; ++i)
			limbs[i]->Update(timestep);
	}

	void RobotTripod::Spawned()
	{
		Dood::Spawned();

		if(is_valid)				// there are some conditions where the Dood may become invalid after a call to Dood::Spawned
		{
			for(int i = 0; i < 3; ++i)
			{
				JointConstraint* use_joints[] =
				{
					(JointConstraint*)constraints[i * 3 + 0],
					(JointConstraint*)constraints[i * 3 + 1],
					(JointConstraint*)constraints[i * 3 + 2]
				};

				RigidBody* use_rigid_bodies[] =
				{
					rigid_bodies[0],
					rigid_bodies[i * 3 + 1],
					rigid_bodies[i * 3 + 2],
					rigid_bodies[i * 3 + 3]
				};

				limbs[i] = new Limb(use_joints, use_rigid_bodies, 3);
				//limb->action = new LAStep(limb);
			}
		}
	}
}
