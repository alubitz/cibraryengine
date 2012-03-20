#include "StdAfx.h"
#include "Plane.h"

#include "Vector.h"
#include "Line.h"

namespace CibraryEngine
{
	Plane::Plane() : normal(), offset(0.0f) { }				// normal vector is degenerate
	Plane::Plane(Vec3 normal_, float offset_) : normal(normal_), offset(offset_) { }

	float Plane::PointDistance(Vec3 point) { return Vec3::Dot(normal, point) - offset; }

	Plane Plane::FromPositionNormal(Vec3 position, Vec3 normal)
	{
		Vec3 unit_normal = Vec3::Normalize(normal);
		return Plane(unit_normal, Vec3::Dot(position, unit_normal));
	}

	Plane Plane::FromTriangleVertices(Vec3 a, Vec3 b, Vec3 c) { return Plane::FromPositionNormal(a, Vec3::Cross(b - a, c - a)); }

	Plane Plane::Reverse(const Plane& plane) { return Plane(-plane.normal, -plane.offset); }

	float Plane::CheckEquality(Plane a, Plane b)
	{
		float magprodsq = a.normal.ComputeMagnitudeSquared() * b.normal.ComputeMagnitudeSquared();		// although the magnitude of the normals SHOULD be 1... maybe somebody did something funky
		float dot = Vec3::Dot(a.normal, b.normal);
		float aparallelness = 1.0f - sqrt((dot * dot) / magprodsq);								// closer to parallel yields smaller values of this
		aparallelness *= aparallelness;

		float a_from_b = b.PointDistance(a.normal * a.offset);
		float b_from_a = a.PointDistance(b.normal * b.offset);
		Vec3 a_on_b = a.normal * (a.offset + a_from_b);
		Vec3 b_on_a = b.normal * (b.offset + b_from_a);
		float distsq1 = (a_on_b - b.normal * b.offset).ComputeMagnitudeSquared();						// coplanar --> same point
		float distsq2 = (b_on_a - a.normal * a.offset).ComputeMagnitudeSquared();						// coplanar --> same point

		return aparallelness + distsq1 + distsq2;
	}

	bool Intersect(Plane a, Plane b, Line& result)
	{
		Vec3 cross = Vec3::Cross(a.normal, b.normal);
		float magsq = cross.ComputeMagnitudeSquared();
		if (magsq == 0)
		{
			// failure! planes did not intersect, or planes were equal
			result = Line(Vec3(), Vec3());					// not a valid line!
			return false;
		}
		float invmag = 1.0f / sqrt(magsq);
		Vec3 line_direction = cross * invmag;
		// using plane a to find intersection (also could try b?)
		Vec3 in_a_toward_edge = Vec3::Normalize(Vec3::Cross(a.normal, line_direction));
		Vec3 point_in_a = a.normal * a.offset;
		float dist = b.PointDistance(point_in_a);
		// seems this number could be either the positive or negative of what we want...
		float unsigned_r = dist * invmag;
		Vec3 positive = point_in_a + in_a_toward_edge * unsigned_r;
		Vec3 negative = point_in_a - in_a_toward_edge * unsigned_r;
		// figure out which one is actually at the intersection (or closest to it)
		float positive_check = Vec2::MagnitudeSquared(a.PointDistance(positive), b.PointDistance(positive));
		float negative_check = Vec2::MagnitudeSquared(a.PointDistance(negative), b.PointDistance(negative));
		// and use that one as a point on the line (for the out value)
		Vec3 point_on_line;
		if (positive_check < negative_check)
			point_on_line = positive;
		else
			point_on_line = negative;
		// success! planes intersected
		result = Line(point_on_line, line_direction);
		return true;
	}
}
