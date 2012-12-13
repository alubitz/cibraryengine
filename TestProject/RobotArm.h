#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class RobotArm : public Dood
	{
		public:

			RobotArm(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void PoseToPhysics(TimingInfo time);
	};
}
