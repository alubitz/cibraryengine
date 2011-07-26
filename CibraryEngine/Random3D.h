#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	class Vec2;
	class Vec3;
	class Quaternion;

	/** Class providing static utility functions for generating random numbers */
	class Random3D
	{
		public:

			/** Returns a random float on the range [0, 1) */
			static float Rand();
			/** Returns a random float on the range [0, max) */
			static float Rand(float max);
			/** Returns a random float on the range [min, max) */
			static float Rand(float min, float max);

			/** Returns a random integer greater than or equal to zero, and less than some obscenely huge number */
			static int RandInt();
			/** Returns a random integer on the range [min, max] */
			static int RandInt(int min, int max);
			/** Returns a random integer on the range [min, max_plus_one) */
			static int RandInt(int max_plus_one);

			/** Returns a random 3-component vector with the specified magnitude */
			static Vec3 RandomNormalizedVector(float len);
			/** Returns a random 2-component vector with the specified magnitude */
			static Vec2 RandomNormalizedVec2(float len);

			/** Returns a quaternion representing a random orientation */
			static Quaternion RandomQuaternionRotation();
	};
}
