#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class PerlinNoise
	{
		private:

			int res, res_sq;

			Vec3* gradients;

		public:

			PerlinNoise(int res);
			~PerlinNoise();

			float Sample(Vec3 uvw);
	};
}
