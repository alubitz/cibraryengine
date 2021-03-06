#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	class Line;

	/** Class representing a plane in 3D space */
	class Plane
	{
		public:

			/** A vector orthogonal to the plane */
			Vec3 normal;
			/** The number of times the normal vector must be added to reach the plane, starting at the coordinate system origin */
			float offset;

			/** Initializes a degenerate plane... don't use this */
			Plane();
			/** Initializes a plane with the specified normal vector and offset */
			Plane(Vec3 normal, float offset);

			/** Returns the signed distance of a point from the plane */
			float PointDistance(Vec3 point);			// Signed distance

			/** Returns a plane which contains the specified point, and which has the specified normal vector */
			static Plane FromPositionNormal(Vec3 position, Vec3 normal);
			/** Returns the plane containing the three given points */
			static Plane FromTriangleVertices(Vec3 a, Vec3 b, Vec3 c);

			/** Determines whether two planes are equal */
			static float CheckEquality(Plane a, Plane b);

			/** Determines whether and where two planes intersect */
			static bool Intersect(Plane a, Plane b, Line& result);
	};
}
