#pragma once

#include "StdAfx.h"
#include "Vector.h"

namespace CibraryEngine
{
	/** Axis-aligned bounding box */
	struct AABB
	{
		Vec3 min, max;

		AABB();												// initializes an inside-out (degenerate) AABB
		AABB(Vec3 point);
		AABB(Vec3 min, Vec3 max);

		bool IsDegenerate();

		bool ContainsPoint(Vec3& point);
		Vec3 GetCenterPoint();

		void Expand(Vec3& point);							// expand to include point; doesn't check if AABB is degenerate!

		static bool IntersectTest(AABB& a, AABB& b);
	};
}
