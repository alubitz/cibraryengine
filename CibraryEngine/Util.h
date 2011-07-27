#pragma once

namespace CibraryEngine
{
	class Vec3;
	class Mat3;

	/** Class providing a few utility functions */
	class Util
	{
		private:
			// This is a class containing static utility methods only - you cannot instantiate it!
			Util() { }
			Util(Util& other) { }
			void operator =(Util& other) { }

		public:

			/** Given the position and velocity of a target, and the muzzle speed of a projectile, returns how many seconds ahead of the target the shooter should aim in order to hit */
			static float LeadTime(Vec3 dx, Vec3 dv, float muzzle_speed);

			/** Given a direction vector, finds an orientation matrix with that as its forward, and the other directions selected randomly */
			static Mat3 FindOrientationZEdge(Vec3 dir);
	};
}
