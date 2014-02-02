#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	class PerlinNoise
	{
		private:

			int res, res_sq;

			Vec3* gradients;

		public:

			PerlinNoise(int res, bool tileable = false);
			~PerlinNoise();

			float Sample(const Vec3& uvw);

			int GetResolution() { return res; }
	};
}
