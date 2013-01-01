#pragma once

#include "StdAfx.h"
#include "LimbAction.h"

namespace Test
{
	class LAClawTest : public LimbAction
	{
	public:

		float claw_timer;
		
		LAClawTest(Limb* limb) : LimbAction(limb), claw_timer(0.0f) { }

		void Update(float timestep)
		{
			claw_timer += timestep;

			limb->joints[limb->joints.size() - 1].desired_pyr = Vec3(sinf(claw_timer), 0, 0);
		}
	};
}
