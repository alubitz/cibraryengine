#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	/** Class representing a sphere in 3D space */
	struct Sphere
	{
		public:

			/** The position of the center of the sphere */
			Vec3 center;
			/** The radius of the sphere */
			float radius;

			/** Initializes a degenerate sphere with radius 0, at the origin */
			Sphere();									// default constructor produces a 0-radius sphere at the origin
			/** Initializes a sphere with the specified center and radius */
			Sphere(Vec3 center, float radius);

			/** Modifies the sphere so that its radius is sufficient to contain the given point, if it is not already sufficient */
			void Expand(Vec3 b);

			/** Returns whether the specified point is within the volume of this sphere (if it's exactly on the radius, it counts) */
			bool ContainsPoint(const Vec3& point) const;

			/** Returns the minimum sphere able to contain the two specified spheres */
			static Sphere Expand(Sphere a, Sphere b);
			/** Returns the minimum sphere able to contain both the sphere and point given */
			static Sphere Expand(Sphere a, Vec3 b);

			/** Determines whether two spheres intersect */
			static bool IntersectTest(Sphere a, Sphere b);
			/** Determines whether one sphere occludes another, given the position of the viewer... this may not be very useful. */
			static bool OcclusionTest(Sphere occluding, Sphere hidden, Vec3 viewer);
	};
}
