#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	class PhysicsWorld;
	struct Vec3;

	class VisionBlocker
	{
		public:

			/**
			 * Checks if there is line-of-sight between two points (based on VisionBlocker objects only);
			 * Returns true if LoS is unobstructed, false otherwise
			 */
			static bool CheckLineOfSight(PhysicsWorld* physics, Vec3 from, Vec3 to);
	};
}
