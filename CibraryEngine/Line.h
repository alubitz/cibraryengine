#pragma once

#include "StdAfx.h"
#include "Vector.h"

namespace CibraryEngine
{
	struct Plane;

	/** Class representing a line in 3D space */
	struct Line
	{
		public:

			/** A point through which the line passes */
			Vec3 origin;
			/** A vector to which the line is parallel */
			Vec3 direction;

			/** Initializes a degenerate line (direction is a null vector) */
			Line();
			/** Initializes a line with the given origin and direction */
			Line(const Vec3& origin, const Vec3& direction);

			/** Determines whether two lines are equal */
			static float CheckEquality(const Line& a, const Line& b);
			/** Determines whether and where a line intersects a plane */
			static bool IntersectPlane(const Line& line, const Plane& plane, Vec3& result);
	};
}
