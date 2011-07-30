#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class Heightfield
	{
		public:

			vector<vector<float> > heights;

			int dim_x, dim_y;

			float horizontal_scale;
			float vertical_scale;

			Vec3 origin;
			Vec2 uv_min, uv_max;



			Heightfield(int w, int h);

			float GetHeightAtUV(float u, float v);
			float Get3DHeight(float x, float z);

			Texture2D* GenerateTexture();
			void GenerateModel(vector<SkinVInfo>& vertex_infos);			// dumps vinfos into list

			static Heightfield* FromTexture(Texture2D* tex, float h_scale, float v_scale);
	};
}
