#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Dood;

	// TODO: absorb as needed: StandingCallback, foot_bones, use_cheaty_physics, posey, PoseToPhysics

	class GaitSelector : public Disposable
	{
		protected:

			Dood* dood;

		public:

			GaitSelector(Dood* dood);

			virtual void Update(float timestep);
	};
}
