#pragma once

#include "StdAfx.h"

#include "LimbAction.h"

namespace Test
{
	class LAStep : public LimbAction
	{
		public:

			float timer;
	
			LAStep(Limb* limb);

			void Update(float timestep);
	};
}
