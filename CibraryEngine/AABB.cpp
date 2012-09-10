#include "StdAfx.h"
#include "AABB.h"

#include "Matrix.h"
#include "DebugLog.h"

#include "Util.h"
#include "Plane.h"

namespace CibraryEngine
{
	using namespace std;


	/*
	 * AABB methods
	 */
	AABB::AABB() : min(), max(-1.0f, -1.0f, -1.0f) { }
	AABB::AABB(Vec3 point) : min(point), max(point) { }
	AABB::AABB(Vec3 min, Vec3 max) : min(min), max(max) { }
	AABB::AABB(Vec3 center, float radius) : min(center.x - radius, center.y - radius, center.z - radius), max(center.x + radius, center.y + radius, center.z + radius) { }

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

	void AABB::Expand(const AABB& aabb)
	{
		min.x = std::min(min.x, aabb.min.x);
		min.y = std::min(min.y, aabb.min.y);
		min.z = std::min(min.z, aabb.min.z);
		max.x = std::max(max.x, aabb.max.x);
		max.y = std::max(max.y, aabb.max.y);
		max.z = std::max(max.z, aabb.max.z);
	}

	AABB AABB::GetTransformedAABB(const Mat4& xform) const
	{
		AABB result = AABB(	xform.TransformVec3(min.x, min.y, min.z, 1.0f));
		result.Expand(		xform.TransformVec3(min.x, min.y, max.z, 1.0f));
		result.Expand(		xform.TransformVec3(min.x, max.y, min.z, 1.0f));
		result.Expand(		xform.TransformVec3(min.x, max.y, max.z, 1.0f));
		result.Expand(		xform.TransformVec3(max.x, min.y, min.z, 1.0f));
		result.Expand(		xform.TransformVec3(max.x, min.y, max.z, 1.0f));
		result.Expand(		xform.TransformVec3(max.x, max.y, min.z, 1.0f));
		result.Expand(		xform.TransformVec3(max.x, max.y, max.z, 1.0f));

		return result;
	}

	bool AABB::IntersectLineSegment(const Vec3& from, const Vec3& to) const
	{
		if(ContainsPoint(from) || ContainsPoint(to))
			return true;

		Ray ray(from, to - from);

		float origin[] = { from.x, from.y, from.z };
		float direction[] = { ray.direction.x, ray.direction.y, ray.direction.z };
		float min_a[] = { min.x, min.y, min.z };
		float max_a[] = { max.x, max.y, max.z };
		float data[] = 
		{
			1,	0,	0,	min.x,	1,	2,
			1,	0,	0,	max.x,	1,	2,
			0,	1,	0,	min.y,	0,	2,
			0,	1,	0,	max.y,	0,	2,
			0,	0,	1,	min.z,	1,	2,
			0,	0,	1,	max.z,	1,	2
		};

		for(int i = 0; i < 6; ++i)
		{
			float* start = &data[i * 6];
			float x = Util::RayPlaneIntersect(ray, Plane(Vec3(start[0], start[1], start[2]), start[3]));
			if(x >= 0 && x <= 1)
			{
				int y_axis = (int)start[4], z_axis = (int)start[5];
				float y = origin[y_axis] + direction[y_axis] * x;
				float z = origin[z_axis] + direction[z_axis] * x;

				if(y >= min_a[y_axis] && y <= max_a[y_axis] && z >= min_a[z_axis] && z <= max_a[z_axis])
					return true;
			}
		}

		return false;
	}
}
