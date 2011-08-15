#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Sun : public LightSource
	{
		public:

			Vec3 position;
			Vec3 color;
			Mat4 view_matrix;

			float distance;
			Mat3 rm;

			VertexBuffer* model;
			Texture2D* texture;

			Sun(Vec3 position, Vec3 color, VertexBuffer* model, Texture2D* texture);

			void SetLight(int which);
			void UnsetLight(int which);

			void Draw();
	};
}
