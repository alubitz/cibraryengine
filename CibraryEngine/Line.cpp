#include "StdAfx.h"
#include "Line.h"

#include "Plane.h"

namespace CibraryEngine
{
	Line::Line(Vec3 origin_, Vec3 direction_) : origin(origin_), direction(direction_) { }

	float Line::CheckEquality(Line a, Line b)
	{
		float magprodsq = a.direction.ComputeMagnitudeSquared() * b.direction.ComputeMagnitudeSquared();
		float dot = Vec3::Dot(a.direction, b.direction);
		float aparallelness = 1.0f - sqrt((dot * dot) / magprodsq);					// closer to parallel yields smaller values of this
		aparallelness *= aparallelness;

		Plane plane_a = Plane::FromPositionNormal(a.origin, a.direction);
		Plane plane_b = Plane::FromPositionNormal(b.origin, b.direction);
		float a_from_b = plane_b.PointDistance(a.origin);
		float b_from_a = plane_a.PointDistance(b.origin);
		Vec3 a_on_b = a.origin + plane_a.normal * a_from_b;
		Vec3 b_on_a = b.origin + plane_b.normal * b_from_a;
		float distsq1 = (a_on_b - b.origin).ComputeMagnitudeSquared();						// colinear --> same point
		float distsq2 = (b_on_a - a.origin).ComputeMagnitudeSquared();						// colinear --> same point
		return aparallelness + distsq1 + distsq2;											// sum of 3 squared quantities... anything big --> big result
	}

	bool Line::IntersectPlane(Line line, Plane plane, Vec3& result)
	{
		Vec3 dir = Vec3::Normalize(line.direction);
		float dir_dot = Vec3::Dot(dir, plane.normal);
		if (dir_dot == 0.0f)
			return false;
		else
		{
			float origin_dot = Vec3::Dot(line.origin, plane.normal);
			float tti = (plane.offset - origin_dot) / dir_dot;

			result = line.origin + dir * tti;
			return true;
		}
	}
}
