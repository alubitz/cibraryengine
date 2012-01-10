#include "StdAfx.h"
#include "MultiMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * MultiMaterial methods
	 */
	MultiMaterial::MultiMaterial() { Clear(); }

	void MultiMaterial::Clear()
	{ 
		types[0] = types[1] = types[2] = types[3] = 0;
		weights[0] = weights[1] = weights[2] = weights[3] = 0; 
	}

	unsigned char MultiMaterial::GetMaterialAmount(unsigned char mat)
	{
		int total = 0;

		for(int i = 0; i < 4; ++i)
			if(types[i] == mat)
				total += weights[i];
		
		return (unsigned char)max(0, min(255, total));
	}

	void MultiMaterial::SetMaterialAmount(unsigned char mat, unsigned char amount)
	{
		int cur = GetMaterialAmount(mat);
		if(cur < amount)
		{
			// increasing the amount
			if(cur > 0)
			{
				// there is already a slot for this material
				for(int i = 1; i < 4; ++i)
					if(types[i] == mat)
						weights[i] = amount;
			}
			else
			{
				// scrap whatever slot has the least stuff in it
				int min = 0;
				for(int i = 1; i < 4; ++i)
					if(weights[i] < weights[min])
						min = i;
				types[min] = mat;
				weights[min] = amount;
			}
		}
		else if(cur > amount)
		{
			// decreasing the amount
			for(int i = 0; i < 4; ++i)
				if(types[i] == mat)
					weights[i] = amount;
		}
	}

	MultiMaterial MultiMaterial::Lerp(MultiMaterial& a, MultiMaterial& b, float mu)
	{
		unsigned char types[8];
		unsigned short weights[8];

		unsigned char mu_char = unsigned char(255.0f * min(1.0f, max(0.0f, mu)));
		unsigned char anti = 255 - mu_char;

		// put all the unique types into the arrays
		int used_types = 0;
		for(int i = 0; i < 8; ++i)
		{
			MultiMaterial& mat = i < 4 ? a : b;
			int index = i % 4;

			if(unsigned short weight = (unsigned short)mat.weights[index])
			{
				unsigned char type = mat.types[index];

				weight *= i < 4 ? anti : mu_char;

				// see if this type is already in the list...
				int j;
				for(j = 0; j < used_types; ++j)
					if(types[j] == type)
					{
						weights[j] += weight;
						break;
					}

				// type is not already in the list, add it
				if(j == used_types)
				{
					types[used_types] = type;
					weights[used_types] = weight;

					++used_types;
				}
			}
		}

		int use_max = min(used_types, 4);

		// pick the top 4 (or less)
		for(int i = 0; i < use_max; ++i)
		{
			for(int j = i + 1; j < used_types; ++j)
				if(weights[j] > weights[i])
				{
					swap(weights[i], weights[j]);
					swap(types[i], types[j]);
				}
		}

		unsigned short highest_weight = weights[0];

		MultiMaterial result;
		for(int i = 0; i < use_max; ++i)
		{
			result.types[i] = types[i];
			result.weights[i] = weights[i] * 255 / highest_weight;
		}

		return result;
	}
}
