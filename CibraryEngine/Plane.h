#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	struct Line;

	/** Class representing a plane in 3D space */
	struct Plane
	{
		public:

			/** A vector orthogonal to the plane */
			Vec3 normal;
			/** The number of times the normal vector must be added to reach the plane, starting at the coordinate system origin */
			float offset;

			/** Initializes a degenerate plane... don't use this */
			Plane();
			/** Initializes a plane with the specified normal vector and offset */
			Plane(const Vec3& normal, float offset);

			/** Returns the signed distance of a point from the plane */
			float PointDistance(const Vec3& point) const;			// Signed distance

			/** Returns a plane which contains the specified point, and which has the specified normal vector */
			static Plane FromPositionNormal(const Vec3& position, const Vec3& normal);
			/** Returns the plane containing the three given points */
			static Plane FromTriangleVertices(const Vec3& a, const Vec3& b, const Vec3& c);

			/** Returns a plane with the same position but with flipped normal vector */
			static Plane Reverse(const Plane& plane);

			/** Determines whether two planes are equal */
			static float CheckEquality(const Plane& a, const Plane& b);

			/** Determines whether and where two planes intersect */
			static bool Intersect(const Plane& a, const Plane& b, Line& result);
	};
}
