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

		AABB()                                 : min(), max(-1.0f, -1.0f, -1.0f) { }			// initializes an inside-out (degenerate) AABB
		AABB(const Vec3& point)                : min(point), max(point) { }
		AABB(const Vec3& min, const Vec3& max) : min(min), max(max) { }
		AABB(const Vec3& center, float radius) : min(center.x - radius, center.y - radius, center.z - radius), max(center.x + radius, center.y + radius, center.z + radius) { }

		bool IsDegenerate() const	{ return min.x > max.x || min.y > max.y || min.z > max.z; }

		bool ContainsPoint(const Vec3& point) const
		{
			return	point.x <= max.x && point.x >= min.x &&
				point.y <= max.y && point.y >= min.y &&
				point.z <= max.z && point.z >= min.z;
		}
		Vec3 GetCenterPoint() const	{ return (min + max) * 0.5f; }

		// expand to include point; doesn't check if AABB is degenerate!
		void Expand(const Vec3& point)
		{
			min.x = std::min(min.x, point.x);
			min.y = std::min(min.y, point.y);
			min.z = std::min(min.z, point.z);
			max.x = std::max(max.x, point.x);
			max.y = std::max(max.y, point.y);
			max.z = std::max(max.z, point.z);
		}

		// expand to include aabb; doesn't check if AABB is degenerate!
		void Expand(const AABB& aabb)
		{
			min.x = std::min(min.x, aabb.min.x);
			min.y = std::min(min.y, aabb.min.y);
			min.z = std::min(min.z, aabb.min.z);
			max.x = std::max(max.x, aabb.max.x);
			max.y = std::max(max.y, aabb.max.y);
			max.z = std::max(max.z, aabb.max.z);
		}

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

		AABB GetTransformedAABB(const Mat4& xform) const;		// gets the AABB of the result of transforming the vertices of this AABB by the specified Mat4

		bool IntersectLineSegment(const Vec3& from, const Vec3& to) const;			// find out if any part of the specified line segment is within this AABB
	};
}
