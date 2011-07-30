#include "StdAfx.h"
#include "Heightfield.h"

namespace Test
{
	/*
	 * Heightfield methods
	 */
	Heightfield::Heightfield(int w, int h) : heights(), dim_x(w), dim_y(h), horizontal_scale(0.05f), vertical_scale(1), origin(), uv_min(), uv_max(1, 1)
	{
		heights.resize(dim_x);
		for(int i = 0; i < dim_x; i++)
		{
			vector<float> col = vector<float>();
			col.resize(dim_y);
			heights.push_back(col);
			for(int j = 0; j < dim_y; j++)
				heights[i].push_back(0);
		}
	}

	float Heightfield::GetHeightAtUV(float u, float v)
	{
		// clamp input to [0,1] range
		u = u < 0 ? 0 : u > 1 ? 1 : u;
		v = v < 0 ? 0 : v > 1 ? 1 : v;

		// scale to dimensions of grid
		u *= dim_x - 1;
		v *= dim_y - 1;

		// round to both high and low
		int x1 = (int)floor(u), x2 = (int)ceil(u), y1 = (int)floor(v), y2 = (int)ceil(v);

		// coefficients for 1-D lerp
		float c_x2 = u - x1, c_x1 = 1 - c_x2, c_y2 = v - y1, c_y1 = 1 - c_y2;
		// coefficients for each of the 4 corners used by the 2-D lerp (products of pairs)
		float c_11 = c_x1 * c_y1, c_12 = c_x1 * c_y2, c_21 = c_x2 * c_y1, c_22 = c_x2 * c_y2;

		// now look up the heights at each vertex
		float h_11 = heights[x1][y1], h_12 = heights[x1][y2], h_21 = heights[x2][y1], h_22 = heights[x2][y2];

		// and use them as weights for the heights
		return c_11 * h_11 + c_12 * h_12 + c_21 * h_21 + c_22 * h_22;
	}

	float Heightfield::Get3DHeight(float x, float z)
	{
		x -= origin.x;
		z -= origin.z;
		return origin.y + vertical_scale * GetHeightAtUV(x / (horizontal_scale * dim_x), z / (horizontal_scale * dim_y));
	}

	Heightfield* Heightfield::FromTexture(Texture2D* tex, float h_scale, float v_scale)
	{
		int w = tex->width, h = tex->height;
		Heightfield* result = new Heightfield(w, h);

		result->horizontal_scale = h_scale;
		result->vertical_scale = v_scale;

		float inv_3x255 = 1.0f / (3.0f * 255.0f);

		for (int x = 0; x < tex->width; x++)
			for (int y = 0; y < tex->height; y++)
			{
				int index = (y * w + x) * 4;
				result->heights[x][y] = ((float)tex->byte_data[index + 0] + (float)tex->byte_data[index + 1] + (float)tex->byte_data[index + 2]) * inv_3x255;
			}

		return result;
	}

	void Heightfield::GenerateModel(vector<SkinVInfo>& vertex_infos)
	{
		int verts = dim_x * dim_y;

		SkinVInfo* vertices = new SkinVInfo[verts];

		int index = 0;

		for (int i = 0; i < dim_x; i++)
			for (int j = 0; j < dim_x; j++)
			{
				float uv_scale = 20.0f;
				Vec3 uv = Vec3(uv_min.x + (float)i / (dim_x - 1) * (uv_max - uv_min).x, 1 - (uv_min.y + (float)j / (dim_y - 1) * (uv_max - uv_min).y), 0) * uv_scale;
				Vec3 xyz = Vec3(i * horizontal_scale, heights[i][j] * vertical_scale, j * horizontal_scale);

				float d_x = vertical_scale / horizontal_scale * (GetHeightAtUV((float)(i + 1) / (dim_x - 1), (float)j / (dim_y - 1)) - GetHeightAtUV((float)(i - 1) / (dim_x - 1), (float)j / (dim_y - 1)));
				float d_y = vertical_scale / horizontal_scale * (GetHeightAtUV((float)i / (dim_x - 1), (float)(j + 1) / (dim_y - 1)) - GetHeightAtUV((float)i / (dim_x - 1), (float)(j - 1) / (dim_y - 1)));
				Vec3 n = Vec3::Normalize(Vec3::Cross(Vec3(0, d_y, 1), Vec3(1, d_x, 0)));

				vertices[index++] = SkinVInfo(xyz, uv, n);
			}

		for (int i = 0; i + 1 < dim_x; i++)
			for (int j = 0; j + 1 < dim_y; j++)
			{
				vertex_infos.push_back(vertices[i * dim_y + j]);
				vertex_infos.push_back(vertices[i * dim_y + (j + 1)]);
				vertex_infos.push_back(vertices[(i + 1) * dim_y + j]);

				vertex_infos.push_back(vertices[(i + 1) * dim_y + j]);
				vertex_infos.push_back(vertices[i * dim_y + (j + 1)]);
				vertex_infos.push_back(vertices[(i + 1) * dim_y + (j + 1)]);
			}

		delete[] vertices;
	}
}
