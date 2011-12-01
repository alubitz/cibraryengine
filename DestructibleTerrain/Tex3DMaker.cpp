#include "StdAfx.h"
#include "Tex3DMaker.h"

#include "PerlinNoise.h"

namespace DestructibleTerrain
{
	/*
	 * Tex3DMaker methods
	 */
	void Tex3DMaker::MakeTex3D()
	{
		int octaves = 4;

		PerlinNoise** noise = new PerlinNoise*[octaves];
		float* vec_coeffs = new float[octaves];
		float* sample_coeffs = new float[octaves];

		PerlinNoise striation(32, true);

		for(int i = 0; i < octaves; i++)
		{
			int octave_res = (8 << i);
			noise[i] = new PerlinNoise(octave_res , true);
			vec_coeffs[i] = float(octave_res - 1);
			sample_coeffs[i] = 1.0f / (2 << i);
		}

		int res = 64;
		int res_sq = res * res;
		int texels = res * res * res;
		int size = texels * 4;

		vector<unsigned char> data;

		Vec3 vec;
		float inv_res_minus_one = 1.0f / (res - 1);
		
		for(int x = 0; x < res; x++)
		{
			vec.x = x * inv_res_minus_one;
			for(int y = 0; y < res; y++)
			{
				vec.y = y * inv_res_minus_one;
				for(int z = 0; z < res; z++)
				{
					vec.z = z * inv_res_minus_one;

					float value = 0.0;
					for(int i = 0; i < octaves; i++)
					{
						float bare_sample = noise[i]->Sample(vec * vec_coeffs[i]);
						float sample_value = bare_sample * 0.5f + 0.5f;

						value += sample_value * sample_coeffs[i];
					}

					value *= striation.Sample(Vec3(1.0f, vec.y * 31 * 0.5f, 1.0f)) * 0.5f + 0.5f;

					unsigned char byte = (unsigned char)max(0.0f, min(255.0f, 255.0f * value));

					data.push_back(byte);
					data.push_back(byte);
					data.push_back(byte);
					data.push_back(255);
				}
			}
		}

		ImageIO::SaveTGA("tex3d.tga", data, res_sq, res);

		for(int i = 0; i < octaves; i++)
			delete noise[i];
		delete[] noise;

		delete[] vec_coeffs;
		delete[] sample_coeffs;
	}
}
