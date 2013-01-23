#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class Limb;

	class RobotTripod : public Dood
	{
		protected:

			void InnerDispose();

		public:

			Limb* limbs[3];

			RobotTripod(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void PoseToPhysics(float timestep);

			void Spawned();
	};
}
