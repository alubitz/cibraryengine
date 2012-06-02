#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class PerlinNoise
	{
		private:

			int res, res_sq;

			Vec3* gradients;

		public:

			PerlinNoise(int res, bool tileable = false);
			~PerlinNoise();

			float Sample(Vec3 uvw);

			int GetResolution() { return res; }
	};
}
