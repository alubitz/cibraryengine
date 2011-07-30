#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Shot;

	class Shootable
	{
		public:
			// Handles how an entity responds to getting shot
			// Return true to allow the shot to hit, false to prevent it from hitting
			virtual bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum) = 0;
	};
}
