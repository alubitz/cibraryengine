#include "StdAfx.h"
#include "AABB.h"

namespace CibraryEngine
{
	using namespace std;


	/*
	 * AABB methods
	 */
	AABB::AABB() : min(), max(-1.0f, -1.0f, -1.0f) { }
	AABB::AABB(Vec3 point) : min(point), max(point) { }
	AABB::AABB(Vec3 min, Vec3 max) : min(min), max(max) { }

	bool AABB::IsDegenerate() const { return min.x > max.x || min.y > max.y || min.z > max.z; }

	bool AABB::ContainsPoint(const Vec3& point) const
	{
		return	point.x <= max.x && point.x >= min.x &&
				point.y <= max.y && point.y >= min.y &&
				point.z <= max.z && point.z >= min.z;
	}

	Vec3 AABB::GetCenterPoint() const { return (min + max) * 0.5f; }

	void AABB::Expand(const Vec3& point)
	{
		min.x = std::min(min.x, point.x);
		min.y = std::min(min.y, point.y);
		min.z = std::min(min.z, point.z);
		max.x = std::max(max.x, point.x);
		max.y = std::max(max.y, point.y);
		max.z = std::max(max.z, point.z);
	}

	bool AABB::IntersectTest(const AABB& a, const AABB& b)
	{
		return	a.min.x <= b.max.x && b.min.x <= a.max.x &&
				a.min.y <= b.max.y && b.min.y <= a.max.y &&
				a.min.z <= b.max.z && b.min.z <= a.max.z;
	}

	
}
