#include "StdAfx.h"
#include "RobotArm.h"

namespace Test
{
	/*
	 * RobotArm methods
	 */
	RobotArm::RobotArm(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) : Dood(gs, model, mphys, pos, team)
	{
		hp *= 1000.0f;
	}

	void RobotArm::PoseToPhysics(TimingInfo time)
	{
		for(unsigned int i = 0; i < constraints.size(); ++i)
		{
			JointConstraint* jc = (JointConstraint*)constraints[i];

			if(i + 1 == constraints.size())
				jc->motor_torque = Vec3(0, 0, 0);
		}
	}
}
