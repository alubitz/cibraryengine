#include "StdAfx.h"

#include "PerlinNoise.h"

namespace DestructibleTerrain
{
	/*
	 * PerlinNoise methods
	 */
	PerlinNoise::PerlinNoise(int res, bool tileable) :
		res(res),
		res_sq(res * res),
		gradients(NULL)
	{
		int n_unique = 256;							// there only need to be this many unique gradients
		Vec3* unique_gradients = new Vec3[n_unique];
		for(int i = 0; i < n_unique; ++i)
			unique_gradients[i] = Random3D::RandomNormalizedVector(1.0f);

		int n = res * res * res;
		gradients = new Vec3[n];
		for(int i = 0; i < n; ++i)
			gradients[i] = unique_gradients[Random3D::RandInt(n_unique)];

		if(tileable)
		{
			int x_span = res * res;
			int res_minus_one = res - 1;
			int rm1_res = res_minus_one * res;
			int rm1_xs = res_minus_one * x_span;

			for(int x = 0; x < res; ++x)
			{
				int xx = x * x_span;

				for(int y = 0; y < res; ++y)
					gradients[xx + y * res] = gradients[xx + y * res + res_minus_one];
				for(int z = 0; z < res; ++z)
					gradients[xx + z] = gradients[xx + rm1_res + z];
			}
			for(int y = 0; y < res; ++y)
			{
				int yy = y * res;
				for(int z = 0; z < res; ++z)
					gradients[yy + z] = gradients[rm1_xs + yy + z];
			}
		}

		delete[] unique_gradients;
	}

	PerlinNoise::~PerlinNoise()
	{
		if(gradients != NULL)
		{
			delete[] gradients;
			gradients = NULL;
		}
	}

	float PerlinNoise::Sample(Vec3 uvw)
	{
		float noise_x = uvw.x - res * floor(uvw.x / res), noise_y = uvw.y - res * floor(uvw.y / res), noise_z = uvw.z - res * floor(uvw.z / res);

		int x1 = (int)floor(noise_x) % res;
		int y1 = (int)floor(noise_y) % res;
		int z1 = (int)floor(noise_z) % res;
		int x2 = (x1 + 1) % res;
		int y2 = (y1 + 1) % res;
		int z2 = (z1 + 1) % res;

		int x1rr = x1 * res_sq, x2rr = x2 * res_sq;
		int y1r = y1 * res, y2r = y2 * res;

		Vec3* eight[] = 
		{
			&gradients[x1rr + y1r + z1],
			&gradients[x1rr + y1r + z2],
			&gradients[x1rr + y2r + z1],
			&gradients[x1rr + y2r + z2],
			&gradients[x2rr + y1r + z1],
			&gradients[x2rr + y1r + z2],
			&gradients[x2rr + y2r + z1],
			&gradients[x2rr + y2r + z2]
		};

		float x_frac = noise_x - x1, y_frac = noise_y - y1, z_frac = noise_z - z1;

		float vert_values[] =
		{
			Vec3::Dot(*eight[0], Vec3(	x_frac,		y_frac,		z_frac		)),
			Vec3::Dot(*eight[1], Vec3(	x_frac,		y_frac,		z_frac - 1	)),
			Vec3::Dot(*eight[2], Vec3(	x_frac,		y_frac - 1,	z_frac		)),
			Vec3::Dot(*eight[3], Vec3(	x_frac,		y_frac - 1,	z_frac - 1	)),
			Vec3::Dot(*eight[4], Vec3(	x_frac - 1,	y_frac,		z_frac		)),
			Vec3::Dot(*eight[5], Vec3(	x_frac - 1,	y_frac,		z_frac - 1	)),
			Vec3::Dot(*eight[6], Vec3(	x_frac - 1,	y_frac - 1,	z_frac		)),
			Vec3::Dot(*eight[7], Vec3(	x_frac - 1,	y_frac - 1,	z_frac - 1	))
		};

		float sx_frac = 3.0f * x_frac * x_frac - 2.0f * x_frac * x_frac * x_frac;
		float sy_frac = 3.0f * y_frac * y_frac - 2.0f * y_frac * y_frac * y_frac;
		float sz_frac = 3.0f * z_frac * z_frac - 2.0f * z_frac * z_frac * z_frac;

		float anti_x = 1.0f - x_frac;
		float anti_y = 1.0f - y_frac;
		float anti_z = 1.0f - z_frac;
		float anti_sx = 3.0f * anti_x * anti_x - 2.0f * anti_x * anti_x * anti_x;
		float anti_sy = 3.0f * anti_y * anti_y - 2.0f * anti_y * anti_y * anti_y;
		float anti_sz = 3.0f * anti_z * anti_z - 2.0f * anti_z * anti_z * anti_z;

		return
			vert_values[0] * anti_sx * anti_sy * anti_sz +
			vert_values[1] * anti_sx * anti_sy * sz_frac +
			vert_values[2] * anti_sx * sy_frac * anti_sz +
			vert_values[3] * anti_sx * sy_frac * sz_frac +
			vert_values[4] * sx_frac * anti_sy * anti_sz +
			vert_values[5] * sx_frac * anti_sy * sz_frac +
			vert_values[6] * sx_frac * sy_frac * anti_sz +
			vert_values[7] * sx_frac * sy_frac * sz_frac;
	}
}
