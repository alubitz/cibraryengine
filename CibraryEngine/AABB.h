#pragma once

#include "StdAfx.h"
#include "Vector.h"

namespace CibraryEngine
{
	struct Mat4;

	/** Axis-aligned bounding box */
	struct AABB
	{
		Vec3 min, max;

		AABB();												// initializes an inside-out (degenerate) AABB
		AABB(Vec3 point);
		AABB(Vec3 min, Vec3 max);
		AABB(Vec3 center, float radius);

		bool IsDegenerate() const;

		bool ContainsPoint(const Vec3& point) const;
		Vec3 GetCenterPoint() const;

		void Expand(const Vec3& point);							// expand to include point; doesn't check if AABB is degenerate!
		void Expand(const AABB& aabb);							// expand to include aabb; doesn't check if AABB is degenerate!

		AABB GetTransformedAABB(const Mat4& xform) const;		// gets the AABB of the result of transforming the vertices of this AABB by the specified Mat4

		static bool IntersectTest(const AABB& a, const AABB& b);
		static bool Intersect(const AABB& a, const AABB& b, AABB& result);			// like IntersectTest, but if true, result contains the overlapping region
	};
}
