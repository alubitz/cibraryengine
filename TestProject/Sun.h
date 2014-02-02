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

			Sun(const Vec3& position, const Vec3& color, VertexBuffer* model, Texture2D* texture);

			void SetLight(int index);
			void UnsetLight(int index);

			void Draw();

			void GenerateShadowMatrices(CameraView& camera, unsigned int n, const float* shadow_region_radii, Mat4* results_out);
	};
}
