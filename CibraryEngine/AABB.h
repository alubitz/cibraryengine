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

		bool IntersectLineSegment(const Vec3& from, const Vec3& to) const;			// find out if any part of the specified line segment is within this AABB

		static bool IntersectTest(const AABB& a, const AABB& b)
		{
			return	a.min.x <= b.max.x && b.min.x <= a.max.x &&
				a.min.y <= b.max.y && b.min.y <= a.max.y &&
				a.min.z <= b.max.z && b.min.z <= a.max.z;
		}
		static bool Intersect(const AABB& a, const AABB& b, AABB& result)			// like IntersectTest, but if true, result contains the overlapping region
		{
			AABB temp;
			temp.min.x = std::max(a.min.x, b.min.x);
			temp.min.y = std::max(a.min.y, b.min.y);
			temp.min.z = std::max(a.min.z, b.min.z);
			temp.max.x = std::min(a.max.x, b.max.x);
			temp.max.y = std::min(a.max.y, b.max.y);
			temp.max.z = std::min(a.max.z, b.max.z);

			if(temp.IsDegenerate())
				return false;
			else
			{
				result = temp;
				return true;
			}
		}
	};
}
