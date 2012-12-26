#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class Limb;

	class RobotArm : public Dood
	{
		protected:

			void InnerDispose();

		public:

			Limb* limb;

			RobotArm(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void PoseToPhysics(float timestep);

			void Spawned();
	};
}
