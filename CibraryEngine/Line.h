#pragma once

#include "StdAfx.h"
#include "Vector.h"

namespace CibraryEngine
{
	class Plane;

	/** Class representing a line in 3D space */
	class Line
	{
		public:

			/** A point through which the line passes */
			Vec3 origin;
			/** A vector to which the line is parallel */
			Vec3 direction;

			/** Initializes a line with the given origin and direction */
			Line(Vec3 origin, Vec3 direction);

			/** Determines whether two lines are equal */
			static float CheckEquality(Line a, Line b);
			/** Determines whether and where a line intersects a plane */
			static bool IntersectPlane(Line line, Plane plane, Vec3& result);
	};
}
