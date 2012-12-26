#pragma once

#include "StdAfx.h"

namespace Test
{
	class Limb;

	class LimbAction
	{
		public:

			Limb* limb;

			LimbAction(Limb* limb);
			virtual ~LimbAction();

			virtual void Update(float timestep) { }			// default implementation does nothing
	};
}
