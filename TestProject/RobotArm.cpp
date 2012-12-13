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
	}
}
