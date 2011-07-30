#include "StdAfx.h"
#include "Util.h"

#include "Vector.h"
#include "Matrix.h"
#include "Random3D.h"

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

	Mat3 Util::FindOrientationZEdge(Vec3 dir)
	{
		Vec3 dir_n = Vec3::Normalize(dir);
		while (true)
		{
			Vec3 up = Random3D::RandomNormalizedVector(1);
			Vec3 right = Vec3::Cross(up, dir_n);
			float magsq = right.ComputeMagnitudeSquared();

			if (magsq == 0)
				continue;

			right /= sqrt(magsq);
			up = Vec3::Cross(dir_n, right);

			float values[] = { right.x, right.y, right.z, up.x, up.y, up.z, dir_n.x, dir_n.y, dir_n.z };
			return Mat3(values);
		}
	}
}
