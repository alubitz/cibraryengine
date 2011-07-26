#pragma once

#include "../CibraryEngine/MathTypes.h"
#include "../CibraryEngine/GraphicsTypes.h"

namespace Test
{
	using namespace CibraryEngine;

	class Sun : public LightSource
	{
		public:

			Vec3 position;
			Vec3 color;
			Mat4 view_matrix;

			Sun(Vec3 position, Vec3 color);

			void SetLight(int which);
			void UnsetLight(int which);
	};
}
