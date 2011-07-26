#include "Util.h"

#include "Vector.h"

namespace CibraryEngine
{
	float Util::LeadTime(Vec3 dx, Vec3 dv, float muzzle_speed)
	{
		float xmag_sq = dx.ComputeMagnitudeSquared();
		float vmag_sq = dv.ComputeMagnitudeSquared();
		float qa = muzzle_speed * muzzle_speed - vmag_sq;
		float qb = 2.0f * Vec3::Dot(dx, dv);
		float qc = -xmag_sq;
		float urad = qb * qb - 4.0f * qa * qc;
		if (urad < 0.0f)
			return -1.0f;
		float root = sqrt(urad);
		float min = -qb - root, max = -qb + root;
		float use = min > 0.0f ? min : max;
		use /= 2.0f * qa;
		return use;
	}
}
